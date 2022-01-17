git clone https://github.com/mavlink/mavlink-gbp-release -b release/noetic/mavlink
mkdir mavlink-gbp-release/build
cd mavlink-gbp-release/build
cmake ..
make
cd ../..
mkdir -p libmavconn/include/mavlink
cp -r mavlink-gbp-release/build/include/v2.0 libmavconn/include/mavlink