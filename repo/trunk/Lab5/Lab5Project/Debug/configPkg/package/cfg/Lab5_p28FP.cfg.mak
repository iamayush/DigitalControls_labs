# invoke SourceDir generated makefile for Lab5.p28FP
Lab5.p28FP: .libraries,Lab5.p28FP
.libraries,Lab5.p28FP: package/cfg/Lab5_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab5/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab5/src/makefile.libs clean

