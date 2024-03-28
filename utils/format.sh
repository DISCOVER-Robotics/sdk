find examples/cpp/airbot_tools/src/ \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cpp' \) -exec clang-format -i {} \;
find ros/src/ \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cpp' \) -exec clang-format -i {} \;
find ros2/src/ \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cpp' \) -exec clang-format -i {} \;