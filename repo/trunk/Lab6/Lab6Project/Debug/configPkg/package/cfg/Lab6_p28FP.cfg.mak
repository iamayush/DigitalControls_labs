# invoke SourceDir generated makefile for Lab6.p28FP
Lab6.p28FP: .libraries,Lab6.p28FP
.libraries,Lab6.p28FP: package/cfg/Lab6_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab6/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab6/src/makefile.libs clean

