# invoke SourceDir generated makefile for lab3.p28FP
lab3.p28FP: .libraries,lab3.p28FP
.libraries,lab3.p28FP: package/cfg/lab3_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab3/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab3/src/makefile.libs clean

