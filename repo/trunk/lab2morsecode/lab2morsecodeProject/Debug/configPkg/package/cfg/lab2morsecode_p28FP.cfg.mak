# invoke SourceDir generated makefile for lab2morsecode.p28FP
lab2morsecode.p28FP: .libraries,lab2morsecode.p28FP
.libraries,lab2morsecode.p28FP: package/cfg/lab2morsecode_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab2morsecode/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab2morsecode/src/makefile.libs clean

