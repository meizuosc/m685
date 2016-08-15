#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>

static int git_commit_id_proc_show(struct seq_file *m, void *v)
{
	unsigned char *commit_id_buffer = CONFIG_GIT_COMMIT_ID;
	int i = 0;

	//seq_printf(m, "%c");
	for(i=0; i<sizeof(CONFIG_GIT_COMMIT_ID); i++) {
		if(commit_id_buffer[i] == '-') {
			seq_printf(m, "\n");
		} else {
			seq_printf(m, "%c", commit_id_buffer[i]);
		}
	}
	return 0;
}

static int git_change_id_proc_show(struct seq_file *m, void *v)
{
	unsigned char *change_id_buffer = CONFIG_GIT_CHANGE_ID;
	int i = 0;

	//seq_printf(m, "%c");
	for(i=0; i<sizeof(CONFIG_GIT_CHANGE_ID); i++) {
		if(change_id_buffer[i] == '-') {
			seq_printf(m, "\n");
		} else {
			seq_printf(m, "%c", change_id_buffer[i]);
		}
	}
	return 0;
}

#define PROC_FOPS_RO(name)	\
	static int name##_proc_open(struct inode *inode, struct file *file)	\
	{									\
		return single_open(file, name##_proc_show, PDE_DATA(inode));	\
	}									\
	static const struct file_operations name##_proc_fops = {		\
		.owner          = THIS_MODULE,					\
		.open           = name##_proc_open,				\
		.read           = seq_read,					\
		.llseek         = seq_lseek,					\
		.release        = single_release,				\
	}

#define PROC_ENTRY(name) {__stringify(name), &name##_proc_fops}
PROC_FOPS_RO(git_commit_id);
PROC_FOPS_RO(git_change_id);

struct pentry {
	const char *name;
	const struct file_operations *fops;
};

const struct pentry git_info_entries[] = {
	PROC_ENTRY(git_commit_id),
	PROC_ENTRY(git_change_id),
};

static int __init proc_git_info_init(void)
{
	struct proc_dir_entry *dir_entry = NULL;
	int i = 0;

	dir_entry = proc_mkdir("git_info", NULL);
	if (!dir_entry) {
		pr_err("GIT_INFO: Failed to create /proc/ entry\n");
		return -ENOMEM;
	}
	
	for (i = 0; i < ARRAY_SIZE(git_info_entries); i++) {
		if (! proc_create(git_info_entries[i].name, S_IRUGO, dir_entry, git_info_entries[i].fops))
			pr_err("GIT_INFO: Failed to create /proc/git_info entry nodes\n");
	}

    return 0;
}
arch_initcall(proc_git_info_init);
