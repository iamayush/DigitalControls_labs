# invoke SourceDir generated makefile for Lab10.p28FP
Lab10.p28FP: .libraries,Lab10.p28FP
.libraries,Lab10.p28FP: package/cfg/Lab10_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab10/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab10/src/makefile.libs clean

