include Param.mk
all:
	echo VERSION_CFG = $(VERSION_CFG)
#	@cd gpio_i2c;   make || exit $$?; cp vvcam_i2c.ko    ../bin/
#	@cd csi;        make || exit $$?; cp bm_csi.ko bm_visys.ko ../bin/
#	@cd sensor;     make || exit $$?; cp vvcam_sensor.ko ../bin/
	@cd isp;        make || exit $$?; cp vvcam_isp_ry.ko    ../bin/
#	@cd soc;        make || exit $$?; cp vvcam_soc.ko    ../bin/
#	@cd dw200;      make || exit $$?; cp vvcam_dw200.ko  ../bin/
#	@cd dec400;     make || exit $$?; cp vvcam_dec400.ko ../bin/
#	@cd vi_pre;     make || exit $$?; cp vi_pre.ko       ../bin/
clean:
#	@cd gpio_i2c;   make clean
#	@cd csi;        make clean
#	@cd sensor;     make clean
	@cd isp;        make clean
#	@cd soc;        make clean
#	@cd dw200;      make clean
#	@cd dec400;     make clean
#	@cd vi_pre;     make clean
cleanall:

