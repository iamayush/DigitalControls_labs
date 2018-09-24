# invoke SourceDir generated makefile for Lab4.p28FP
Lab4.p28FP: .libraries,Lab4.p28FP
.libraries,Lab4.p28FP: package/cfg/Lab4_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab4/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab4/src/makefile.libs clean

