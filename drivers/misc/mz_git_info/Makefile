GIT_INFO_OBJ_DIR := $(objtree)/drivers/misc/mz_git_info

subdir-ccflags-y += -DCONFIG_GIT_COMMIT_ID=\"`cd $(srctree); git log | egrep 'commit ' | egrep '\w{40}$$' -o | head -10 | tr '\n' '-'`\"
subdir-ccflags-y += -DCONFIG_GIT_CHANGE_ID=\"`cd $(srctree); git log | egrep 'Change-Id' | egrep '\w{41}$$' -o | head -10 | tr '\n' '-'`\"
#$(warning $(PWD))
#$(warning $(subdir))
#$(warning $(shell printenv))

#Delete Output File
$(shell rm $(GIT_INFO_OBJ_DIR)/git_info.o)

obj-y += git_info.o

