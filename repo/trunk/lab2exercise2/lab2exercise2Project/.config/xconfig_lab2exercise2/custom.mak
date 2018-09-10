## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,28FP linker.cmd package/cfg/lab2exercise2_p28FP.o28FP

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/lab2exercise2_p28FP.xdl
	$(SED) 's"^\"\(package/cfg/lab2exercise2_p28FPcfg.cmd\)\"$""\"C:/ayush2_nigam4/repo/trunk/lab2exercise2/lab2exercise2Project/.config/xconfig_lab2exercise2/\1\""' package/cfg/lab2exercise2_p28FP.xdl > $@
	-$(SETDATE) -r:max package/cfg/lab2exercise2_p28FP.h compiler.opt compiler.opt.defs
