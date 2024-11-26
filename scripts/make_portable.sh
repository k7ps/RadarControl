#!/bin/bash
CURRENT_DIR="/home/k1ps/VSCodeProjects/RadarControl"
BIN_PATH="$CURRENT_DIR/build/tools/bin/Tools"
PORTABLE_PATH="/home/k1ps/Desktop"
PORTABLE_DIR="RadarControlApp"
LAUNCH_SCRIPT="run.sh"

rm -r -f $PORTABLE_PATH/$PORTABLE_DIR

mkdir -p $PORTABLE_PATH/$PORTABLE_DIR
cd $PORTABLE_PATH/$PORTABLE_DIR

cp -u -r $CURRENT_DIR/params .

mkdir -p bin
cp -u $BIN_PATH bin

mkdir -p lib
ldd $BIN_PATH | grep "=>" | awk '{print $3}' | xargs -I '{}' cp '{}' lib/

# generating launch script
cat << EOF > $PORTABLE_PATH/$PORTABLE_DIR/$LAUNCH_SCRIPT
#!/bin/bash
export LD_LIBRARY_PATH=\$PWD/lib:\$LD_LIBRARY_PATH
cd bin/
./Tools

EOF

chmod +x $PORTABLE_PATH/$PORTABLE_DIR/$LAUNCH_SCRIPT

cd ..
rm -f $PORTABLE_DIR.zip
zip -r -q $PORTABLE_DIR.zip $PORTABLE_DIR

echo "Success"
