source env-imx8.sh

make imx_v8_defconfig
make Image dtbs modules

sudo rm -r MODULES
mkdir MODULES
make -j16 modules_install CROSS_COMPILE=$CROSS_COMPILER ARCH=arm64 INSTALL_MOD_PATH=./MODULES


cp arch/arm64/boot/Image /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk-isi0-ov5640.dtbo /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk-isp0-ov5640.dtbo /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk-isi0-alvium.dtbo /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk-alvium.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-sm2sv2-alvium.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-sm2s.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mm-sm2s.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-sm2sv2-alvium-gmsl2.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-sm2sv2-alvium-csi0-gmsl2.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-sm2sv2-alvium-csi1-gmsl2.dtb /tftp-folder/

sudo cp -rf MODULES/lib/modules/* /targetfs/lib/modules/
