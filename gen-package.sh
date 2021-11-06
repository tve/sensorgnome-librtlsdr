#! /bin/bash -e
export DESTDIR=build-temp
rm -rf $DESTDIR
mkdir $DESTDIR

# create dockcross script
echo "Generating dockercross script"
IMG=tvoneicken/sensorgnome-dockcross:armv7-rpi-buster-main
docker run $IMG >sensorgnome-dockcross
chmod +x sensorgnome-dockcross

# build process
# evaluate DESTDIR within container to an absolute path, else things break
CDESTDIR=$(./sensorgnome-dockcross -i $IMG bash -c "cd $DESTDIR; pwd" | tr -d '\r')
echo "Autoreconf"
./sensorgnome-dockcross -i $IMG \
    autoreconf -i
echo "Configure"
# using bash -c so $xxx are expanded within the dockcross container
conf_flags='LIBUSB_CFLAGS=-I$CROSS_ROOT/$CROSS_TRIPLE/sysroot/usr/include/libusb-1.0 '
conf_flags+='LIBUSB_LIBS=-L$CROSS_ROOT/$CROSS_TRIPLE/sysroot/usr/lib/ '
conf_flags+='LDFLAGS=-lusb-1.0 '
./sensorgnome-dockcross -i $IMG \
    bash -x -c './configure --build=x86_64-unknown-linux-gnu --host $CROSS_TRIPLE'" $conf_flags"
make clean
echo ""
echo "Running make"
./sensorgnome-dockcross -i $IMG \
    make -j4 install DESTDIR=$CDESTDIR STRIP=\$CROSS_TRIPLE-strip
echo "Libtool finish"
./sensorgnome-dockcross -i $IMG \
    libtool --finish $CDESTDIR

# Boilerplate package generation
cp -r DEBIAN $DESTDIR
sed -e "/^Version/s/:.*/: $(date +%Y.%j)/" -i $DESTDIR/DEBIAN/control # set version: YYYY.DDD
mkdir -p packages
dpkg-deb --root-owner-group --build $DESTDIR packages
# dpkg-deb --contents packages
ls -lh packages
