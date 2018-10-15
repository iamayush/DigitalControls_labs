# invoke SourceDir generated makefile for Lab7.p28FP
Lab7.p28FP: .libraries,Lab7.p28FP
.libraries,Lab7.p28FP: package/cfg/Lab7_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab7/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab7/src/makefile.libs clean

