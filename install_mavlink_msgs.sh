git clone https://github.com/mavlink/mavlink-gbp-release -b release/noetic/mavlink
cp custom_definitions/qrotor.xml mavlink-gbp-release/message_definitions/v1.0/
mkdir mavlink-gbp-release/build
cd mavlink-gbp-release/build
cmake ..
make
cd ../..
mkdir -p libmavconn/include/mavlink
cp -r mavlink-gbp-release/build/include/v2.0 include/mavlink