# invoke SourceDir generated makefile for Lab11.p28FP
Lab11.p28FP: .libraries,Lab11.p28FP
.libraries,Lab11.p28FP: package/cfg/Lab11_p28FP.xdl
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab11/src/makefile.libs

clean::
	$(MAKE) -f C:\ayush2_nigam4\repo\trunk\Lab11/src/makefile.libs clean

