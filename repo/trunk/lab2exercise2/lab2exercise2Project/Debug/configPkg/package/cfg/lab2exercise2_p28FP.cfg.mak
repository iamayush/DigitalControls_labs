# invoke SourceDir generated makefile for lab2exercise2.p28FP
lab2exercise2.p28FP: .libraries,lab2exercise2.p28FP
.libraries,lab2exercise2.p28FP: package/cfg/lab2exercise2_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab2exercise2/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\lab2exercise2/src/makefile.libs clean

