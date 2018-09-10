# invoke SourceDir generated makefile for lab2switchchecker.p28FP
lab2switchchecker.p28FP: .libraries,lab2switchchecker.p28FP
.libraries,lab2switchchecker.p28FP: package/cfg/lab2switchchecker_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab2switchchecker/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab2switchchecker/src/makefile.libs clean

