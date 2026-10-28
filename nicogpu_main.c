// SPDX-License-Identifier: GPL-2.0
// nicogpu_main.c -- Nicogpu PCIe driver: probe, BAR
// mapping, MSI, drm_device lifetime.
//
// Device 10ee:9031 (XDMA AXI-Bridge PF0).  BAR0 1 MiB RM regs, BAR1 64 KiB shell
// regs, BAR2 256 MiB RM memory window.  The XDMA bridge exposes one usr_irq_req
// line, so shell MSI vectors 0 (RM) and 1 (shell) are OR-merged: whichever
// vector the kernel grants (1 or 2), the handler reads BOTH SHELL_IRQ_STATUS
// and the RM IRQ_STATUS, W1Cs what it saw and EOIs both vectors
// (SHELL_IRQ_EOI).
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <drm/drm_drv.h>
#include "nicogpu_drv.h"

static int vid = NICOGPU_PCI_VENDOR;
static int did = NICOGPU_PCI_DEVICE;
module_param(vid, int, 0444);
MODULE_PARM_DESC(vid, "PCI vendor id to bind (default 0x10ee)");
static bool bar_lock;
module_param(bar_lock, bool, 0444);
MODULE_PARM_DESC(bar_lock, "serialise BAR0/BAR1 register accesses (default off; only needed with a shell built before the ng_bar_router read-order fix)");
module_param(did, int, 0444);
MODULE_PARM_DESC(did, "PCI device id to bind (default 0x9031)");

static irqreturn_t nicogpu_isr(int irq, void *data)
{
	struct nicogpu_dev *x = data;
	u32 sh = ng_rd(x, NICOGPU_BAR_SHELL, SH_IRQ_STATUS);
	u32 rm;

	/* An isolated / decoupled RM reads all-ones (nicogpu_regs.h, NICOGPU_BAR_CTRL): no RM_STATUS probe first,
	 * the all-ones check below is the whole test (one non-posted read less per interrupt) */
	rm = ng_rd(x, NICOGPU_BAR_CTRL, RM_IRQ_STATUS);
	if (rm == 0xFFFFFFFFu)
		rm = 0;

	atomic64_inc(&x->irq_count);
	atomic_or(sh, &x->irq_shell_latched);
	atomic_or(rm, &x->irq_rm_latched);
	dev_dbg(&x->pdev->dev, "MSI irq=%d shell_irq=%08x rm_irq=%08x\n", irq, sh, rm);

	/* W1C the sources first; dropping these while EOIing re-fires forever
	 * (2026-09-27 board storm, 39356 interrupts). */
	if (sh)
		ng_wr(x, NICOGPU_BAR_SHELL, SH_IRQ_STATUS, sh);
	if (rm)
		ng_wr(x, NICOGPU_BAR_CTRL, RM_IRQ_STATUS, rm);
	ng_wr(x, NICOGPU_BAR_SHELL, SH_IRQ_EOI, 3);	/* re-arm both vectors */
	if (rm)
		nicogpu_sched_irq(x, rm);	/* fence / CP error / MMU fault of the RM */
	return IRQ_HANDLED;
}

void nicogpu_fill_info(struct nicogpu_dev *x, struct nicogpu_info *i)
{
	int b;

	i->abi_version = NICOGPU_ABI_VERSION;
	i->msi_vectors = x->nvec;
	i->shell_id = ng_rd(x, NICOGPU_BAR_SHELL, SH_ID);
	i->shell_version = ng_rd(x, NICOGPU_BAR_SHELL, SH_VERSION);
	i->shell_caps = ng_rd(x, NICOGPU_BAR_SHELL, SH_CAPS);
	i->shell_status = ng_rd(x, NICOGPU_BAR_SHELL, SH_STATUS);
	i->shell_uid_lo = ng_rd(x, NICOGPU_BAR_SHELL, SH_UID_LO);
	i->shell_uid_hi = ng_rd(x, NICOGPU_BAR_SHELL, SH_UID_HI);
	i->rm_status = ng_rd(x, NICOGPU_BAR_SHELL, RM_STATUS);
	for (b = 0; b < 4; b++)
		i->rm_info[b] = ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO0 + 4 * b);
	i->clk_freq_khz[0] = ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_CORE);
	i->clk_freq_khz[1] = ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_L2);
	i->clk_freq_khz[2] = ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_TS);
	i->clk_freq_khz[3] = ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_CORE2X);
	i->clk_status = ng_rd(x, NICOGPU_BAR_SHELL, CLK_STATUS);
	for (b = 0; b < NICOGPU_NUM_BARS; b++)
		i->bar_len[b] = pci_resource_len(x->pdev, b);
	i->irq_count = atomic64_read(&x->irq_count);
	i->irq_shell_latched = atomic_read(&x->irq_shell_latched);
	i->irq_rm_latched = atomic_read(&x->irq_rm_latched);
}

static int nicogpu_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct nicogpu_dev *x;
	u32 shid, ver;
	int i, rc;

	/* the nicogpu_dev embeds the drm_device (first member); it lives until the last drm_dev_put(): the devres action
	 * of this call drops the driver's reference after remove(), open files / GEM objects / vmas hold their own */
	x = devm_drm_dev_alloc(&pdev->dev, &nicogpu_drm_driver, struct nicogpu_dev, drm);
	if (IS_ERR(x))
		return PTR_ERR(x);
	x->pdev = pdev;
	x->bar_lock = bar_lock;
	spin_lock_init(&x->bar0_lock);
	x->mapping = x->drm.anon_inode->i_mapping;	/* every GEM / debug mapping of every file lives in it */
	mutex_init(&x->dbuf_lock);
	mutex_init(&x->ts_lock);
	pci_set_drvdata(pdev, x);

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;
	for (i = 0; i < NICOGPU_NUM_BARS; i++) {
		if (!(pci_resource_flags(pdev, i) & IORESOURCE_MEM)) {
			dev_err(&pdev->dev, "BAR%d missing\n", i);
			return -ENODEV;
		}
	}
	/* sizes are fixed by gen_ip.tcl; a mismatch means a different build */
	if (pci_resource_len(pdev, 0) != SZ_1M || pci_resource_len(pdev, 1) != SZ_64K ||
	    pci_resource_len(pdev, 2) != SZ_256M)
		dev_warn(&pdev->dev, "unexpected BAR sizes %llx/%llx/%llx (want 100000/10000/10000000)\n",
			 (u64)pci_resource_len(pdev, 0), (u64)pci_resource_len(pdev, 1),
			 (u64)pci_resource_len(pdev, 2));
	rc = pcim_iomap_regions(pdev, 0x7, "nicogpu");
	if (rc)
		return rc;
	for (i = 0; i < NICOGPU_NUM_BARS; i++)
		x->bar[i] = pcim_iomap_table(pdev)[i];

	shid = ng_rd(x, NICOGPU_BAR_SHELL, SH_ID);
	ver = ng_rd(x, NICOGPU_BAR_SHELL, SH_VERSION);
	if (shid != SH_ID_MAGIC) {
		dev_err(&pdev->dev, "SHELL_ID %08x != XSHL (link down or wrong build)\n", shid);
		return -ENODEV;
	}
	if ((ver & 0xFFFF) < 2)
		dev_warn(&pdev->dev, "SHELL_REV %u < 2: no CLK_FREQ meters / FENCE_STUCK\n", ver & 0xFFFF);

	pci_set_master(pdev);
	/* the shell's m_axi_host issues 47-bit bus addresses */
	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(47));
	if (rc)
		dev_warn(&pdev->dev, "47-bit DMA mask rejected (%d): DMABUF unusable\n", rc);

	x->nvec = pci_alloc_irq_vectors(pdev, 1, 2, PCI_IRQ_MSI);
	if (x->nvec < 0)
		return x->nvec;
	dev_info(&pdev->dev, "MSI vectors granted: %d (shell merges vec 0|1)\n", x->nvec);
	for (i = 0; i < x->nvec; i++) {
		/* explicit request_irq/free_irq, NOT devm: devres runs after
		 * ->remove(), so the /proc/irq/N entry would still exist when
		 * pci_free_irq_vectors() runs (remove_proc_entry WARNING). */
		rc = request_irq(pci_irq_vector(pdev, i), nicogpu_isr, 0, "nicogpu", x);
		if (rc) {
			while (i--)
				free_irq(pci_irq_vector(pdev, i), x);
			pci_free_irq_vectors(pdev);
			return rc;
		}
	}
	ng_wr(x, NICOGPU_BAR_SHELL, SH_IRQ_ENABLE, SH_IRQ_ALL);

	rc = nicogpu_mm_init(x);
	if (rc) {
		dev_err(&pdev->dev, "memory manager init failed: %d\n", rc);
		goto err_irq;
	}
	nicogpu_sched_init(x);
	nicogpu_dfx_init(x);
	rc = drm_dev_register(&x->drm, 0);	/* /dev/dri/renderD*: from here on files may open */
	if (rc) {
		dev_err(&pdev->dev, "drm_dev_register failed: %d\n", rc);
		nicogpu_sched_fini(x);
		nicogpu_dfx_fini(x);
		nicogpu_mm_fini(x);
		goto err_irq;
	}
	nicogpu_debugfs_init(x);
	dev_info(&pdev->dev, "nicogpu ready: shell rev %u iface %04x status %08x, render node renderD%d\n",
		 ver & 0xFFFF, ver >> 16, ng_rd(x, NICOGPU_BAR_SHELL, SH_STATUS), x->drm.render->index);
	return 0;

err_irq:
	ng_wr(x, NICOGPU_BAR_SHELL, SH_IRQ_ENABLE, 0);
	for (i = 0; i < x->nvec; i++)
		free_irq(pci_irq_vector(pdev, i), x);
	pci_free_irq_vectors(pdev);
	return rc;
}

static void nicogpu_remove(struct pci_dev *pdev)
{
	struct nicogpu_dev *x = pci_get_drvdata(pdev);
	int i;

	/* reverse probe order.  drm_dev_unplug(): wait for the ioctls / mmaps in flight, fail new ones with -ENODEV, remove
	 * the device nodes (open files stay open); then take the CPU mappings away (BAR windows, VRAM BOs: they would read
	 * all-ones, GTT / DMA buffers are freed below) and tear down what the files still own while the device is alive */
	nicogpu_debugfs_fini(x);
	drm_dev_unplug(&x->drm);
	unmap_mapping_range(x->mapping, 0, 0, 1);
	nicogpu_mm_remove_files(x);	/* queues / VMs still owned by open files (their BOs are dropped at close) */
	nicogpu_dma_free(x);
	nicogpu_sched_fini(x);		/* CP off, work items stopped, ring/fence page freed */
	nicogpu_dfx_fini(x);
	nicogpu_mm_fini(x);
	ng_wr(x, NICOGPU_BAR_SHELL, SH_IRQ_ENABLE, 0);
	for (i = 0; i < x->nvec; i++)
		free_irq(pci_irq_vector(pdev, i), x);
	pci_free_irq_vectors(pdev);
}

static struct pci_device_id nicogpu_ids[] = {
	{ PCI_DEVICE(NICOGPU_PCI_VENDOR, NICOGPU_PCI_DEVICE) },
	{ }
};
MODULE_DEVICE_TABLE(pci, nicogpu_ids);

static struct pci_device_id nicogpu_dyn_ids[2];	/* vid=/did= overridable */
static struct pci_driver nicogpu_drv = {
	.name = "nicogpu",
	.id_table = nicogpu_dyn_ids,
	.probe = nicogpu_probe,
	.remove = nicogpu_remove,
};

static int __init nicogpu_init(void)
{
	nicogpu_dyn_ids[0] = nicogpu_ids[0];
	nicogpu_dyn_ids[0].vendor = vid;
	nicogpu_dyn_ids[0].device = did;
	return pci_register_driver(&nicogpu_drv);
}
static void __exit nicogpu_exit(void)
{
	pci_unregister_driver(&nicogpu_drv);
}
module_init(nicogpu_init);
module_exit(nicogpu_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NicoWorks Heathcliff GPU PCIe driver (DRM render node)");
