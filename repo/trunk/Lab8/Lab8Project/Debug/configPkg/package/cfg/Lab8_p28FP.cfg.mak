# invoke SourceDir generated makefile for Lab8.p28FP
Lab8.p28FP: .libraries,Lab8.p28FP
.libraries,Lab8.p28FP: package/cfg/Lab8_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab8/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab8/src/makefile.libs clean

