// SPDX-License-Identifier: GPL-2.0
// nicogpu_debugfs.c -- /sys/kernel/debug/nicogpu-<bdf>/{shell,clk,irq,bars,sched,icap}, read-only, and (root only)
// the write-only operator files `reset` ("1" CP_CTRL.RESET, "2" firmware reload, "3" GPU-zone reset), `pr_load`
// (firmware name of an NGPR container for request_firmware()) and `dump` (take a devcoredump).
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "nicogpu_drv.h"

static int shell_show(struct seq_file *m, void *p)
{
	struct nicogpu_dev *x = m->private;
	u32 st = ng_rd(x, NICOGPU_BAR_SHELL, SH_STATUS), rs = ng_rd(x, NICOGPU_BAR_SHELL, RM_STATUS);
	u32 v = ng_rd(x, NICOGPU_BAR_SHELL, SH_VERSION), caps = ng_rd(x, NICOGPU_BAR_SHELL, SH_CAPS);

	seq_printf(m, "SHELL_ID       %08x\n", ng_rd(x, NICOGPU_BAR_SHELL, SH_ID));
	seq_printf(m, "SHELL_VERSION  %08x (rev %u, iface %04x)\n", v, v & 0xFFFF, v >> 16);
	seq_printf(m, "SHELL_CAPS     %08x (icap %u clk_drp %u jtag_axi %u mem_present %x msi_vec %u)\n",
		   caps, !!(caps & SH_CAPS_ICAP), !!(caps & SH_CAPS_CLK_DRP), !!(caps & SH_CAPS_JTAG_AXI),
		   caps >> SH_CAPS_MEM_SHIFT & 0xF, caps >> SH_CAPS_MSI_SHIFT & 0xF);
	seq_printf(m, "SHELL_UID      %08x_%08x\n", ng_rd(x, NICOGPU_BAR_SHELL, SH_UID_HI),
		   ng_rd(x, NICOGPU_BAR_SHELL, SH_UID_LO));
	seq_printf(m, "SHELL_STATUS   %08x (si_done %u si_err %u mmcm %u calib %x link %u fence_stuck %u)\n",
		   st, !!(st & SH_ST_SI_DONE), !!(st & SH_ST_SI_ERR), !!(st & SH_ST_MMCM), st >> SH_ST_CALIB_SHIFT & 0xF,
		   !!(st & SH_ST_LINK), !!(st & SH_ST_FENCE_STUCK));
	seq_printf(m, "RM_STATUS      %08x (decoupled %u in_reset %u ready %u info_valid %u iface_ok %u mem_ok %u)\n",
		   rs, !!(rs & RM_ST_DECOUPLED), !!(rs & RM_ST_IN_RESET), !!(rs & RM_ST_READY), !!(rs & RM_ST_INFO_VALID),
		   !!(rs & RM_ST_IFACE_OK), !!(rs & RM_ST_MEM_OK));
	seq_printf(m, "RM_CTRL        %08x\n", ng_rd(x, NICOGPU_BAR_SHELL, RM_CTRL));
	seq_printf(m, "RM_INFO        %08x %08x %08x %08x\n",
		   ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO3), ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO2),
		   ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO1), ng_rd(x, NICOGPU_BAR_SHELL, RM_INFO0));
	seq_printf(m, "RM_ERR_COUNT   %u\n", ng_rd(x, NICOGPU_BAR_SHELL, RM_ERR_COUNT));
	seq_printf(m, "PR_STATUS      %08x\n", ng_rd(x, NICOGPU_BAR_SHELL, PR_STATUS));
	seq_printf(m, "BAR0_ID        %08x (ffffffff = isolated)\n",
		   (rs & RM_ST_DECOUPLED) ? 0xFFFFFFFFu : ng_rd(x, NICOGPU_BAR_CTRL, RM_ID));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(shell);

static int clk_show(struct seq_file *m, void *p)
{
	struct nicogpu_dev *x = m->private;

	seq_printf(m, "CLK_CTRL %08x CLK_STATUS %08x (locked/gated bits 0/1)\n",
		   ng_rd(x, NICOGPU_BAR_SHELL, CLK_CTRL), ng_rd(x, NICOGPU_BAR_SHELL, CLK_STATUS));
	seq_printf(m, "CLK_FREQ core %u kHz  core2x %u kHz  l2 %u kHz  ts %u kHz\n",
		   ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_CORE), ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_CORE2X),
		   ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_L2), ng_rd(x, NICOGPU_BAR_SHELL, CLK_FREQ_TS));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(clk);

static int irq_show(struct seq_file *m, void *p)
{
	struct nicogpu_dev *x = m->private;

	seq_printf(m, "msi_vectors   %d\n", x->nvec);
	seq_printf(m, "irq_count     %lld\n", atomic64_read(&x->irq_count));
	seq_printf(m, "shell_latched %08x\n", atomic_read(&x->irq_shell_latched));
	seq_printf(m, "rm_latched    %08x\n", atomic_read(&x->irq_rm_latched));
	seq_printf(m, "SHELL_IRQ_STATUS %08x ENABLE %08x\n", ng_rd(x, NICOGPU_BAR_SHELL, SH_IRQ_STATUS),
		   ng_rd(x, NICOGPU_BAR_SHELL, SH_IRQ_ENABLE));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(irq);

static int bars_show(struct seq_file *m, void *p)
{
	struct nicogpu_dev *x = m->private;
	int i;

	for (i = 0; i < NICOGPU_NUM_BARS; i++)
		seq_printf(m, "BAR%d start %llx len %llx flags %lx mapped %p\n", i,
			   (u64)pci_resource_start(x->pdev, i), (u64)pci_resource_len(x->pdev, i),
			   pci_resource_flags(x->pdev, i), x->bar[i]);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bars);

static int icap_show(struct seq_file *m, void *p)
{
	nicogpu_dfx_icap_show(m->private, m);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(icap);

static int sched_show(struct seq_file *m, void *p)
{
	nicogpu_sched_debugfs_show(m->private, m);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sched);

/* write-only operator files: the string is the command */
static ssize_t op_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos, int which)
{
	struct nicogpu_dev *x = f->private_data;
	char buf[64];
	long v;
	int rc;

	if (!len || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	if (buf[len - 1] == '\n')
		buf[len - 1] = 0;
	switch (which) {
	case 0:
		if (kstrtol(buf, 0, &v))
			return -EINVAL;
		rc = nicogpu_sched_reset(x, (u32)v);
		break;
	case 1:
		rc = nicogpu_dfx_pr_load_name(x, buf);
		break;
	default:
		nicogpu_dump(x, XDD_R_MANUAL);
		rc = 0;
		break;
	}
	return rc ? rc : (ssize_t)len;
}

static ssize_t reset_write(struct file *f, const char __user *u, size_t n, loff_t *p)
{
	return op_write(f, u, n, p, 0);
}

static ssize_t pr_load_write(struct file *f, const char __user *u, size_t n, loff_t *p)
{
	return op_write(f, u, n, p, 1);
}

static ssize_t dump_write(struct file *f, const char __user *u, size_t n, loff_t *p)
{
	return op_write(f, u, n, p, 2);
}

static const struct file_operations reset_fops = { .owner = THIS_MODULE, .open = simple_open, .write = reset_write };
static const struct file_operations pr_load_fops = { .owner = THIS_MODULE, .open = simple_open, .write = pr_load_write };
static const struct file_operations dump_fops = { .owner = THIS_MODULE, .open = simple_open, .write = dump_write };

void nicogpu_debugfs_init(struct nicogpu_dev *x)
{
	char name[40];

	snprintf(name, sizeof(name), "nicogpu-%s", pci_name(x->pdev));
	x->dbg = debugfs_create_dir(name, NULL);
	debugfs_create_file("shell", 0400, x->dbg, x, &shell_fops);
	debugfs_create_file("clk", 0400, x->dbg, x, &clk_fops);
	debugfs_create_file("irq", 0400, x->dbg, x, &irq_fops);
	debugfs_create_file("bars", 0400, x->dbg, x, &bars_fops);
	debugfs_create_file("sched", 0400, x->dbg, x, &sched_fops);
	debugfs_create_file("icap", 0400, x->dbg, x, &icap_fops);	/* reading it runs ICAP readbacks */
	debugfs_create_file("reset", 0200, x->dbg, x, &reset_fops);
	debugfs_create_file("pr_load", 0200, x->dbg, x, &pr_load_fops);
	debugfs_create_file("dump", 0200, x->dbg, x, &dump_fops);
}

void nicogpu_debugfs_fini(struct nicogpu_dev *x)
{
	debugfs_remove_recursive(x->dbg);
	x->dbg = NULL;
}
