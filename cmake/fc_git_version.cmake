# =============================================================================
#  fc_git_version.cmake — sinh fc_git_version.h tu git, chay o MOI LAN BUILD.
#
#  Goi bang:  cmake -DSRC=<repo> -DOUT=<file.h> -P fc_git_version.cmake
#
#  Vi sao chay moi lan build chu khong chi luc configure:
#    AUTOPILOT_VERSION.flight_custom_version phai cho biet CHINH XAC Pi dang noi
#    chuyen voi ban build nao (GIAO_UOC_FC_ROS2.md muc 9.5, 10.1). Lay hash luc
#    configure thi commit xong ma khong configure lai la hash cu nam im trong
#    firmware — dung kieu sai khien moc phien ban vo nghia.
#
#  Chi GHI file khi noi dung doi, de khong kich build lai toan bo moi lan.
# =============================================================================

set(hash "0000000000000000000000000000000000000000")
set(dirty 1)

execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY "${SRC}"
    OUTPUT_VARIABLE git_out
    RESULT_VARIABLE git_rc
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)

if(git_rc EQUAL 0 AND git_out MATCHES "^[0-9a-f]+$")
    set(hash "${git_out}")
    # Cay lam viec co thay doi chua commit -> hash KHONG dai dien cho ban build.
    execute_process(
        COMMAND git diff --quiet HEAD --
        WORKING_DIRECTORY "${SRC}"
        RESULT_VARIABLE diff_rc
        ERROR_QUIET)
    if(diff_rc EQUAL 0)
        set(dirty 0)
    endif()
endif()

# 8 byte dau cua SHA, dang mang byte cho mavlink_msg_autopilot_version_pack().
#
# THU TU BYTE (hop dong 1.2, GIAO_UOC muc 9.5 va 11.1 #9): 16 chu so hex duoc
# coi la MOT so uint64 va ghi little-endian — byte THAP nhat truoc. MAVROS doc
# 8 byte thanh uint64 little-endian roi in hex, nen cach nay lam log MAVROS in
# DUNG chuoi hash. Truoc 1.2 ghi theo thu tu chuoi va MAVROS in nguoc.
#   hash 8b9b35f7b2223c3a  ->  tren day 3a 3c 22 b2 f7 35 9b 8b
string(SUBSTRING "${hash}" 0 16 h16)
set(bytes "")
foreach(i RANGE 14 0 -2)
    string(SUBSTRING "${h16}" ${i} 2 b)
    string(APPEND bytes "0x${b}, ")
endforeach()

set(content "/* Tu sinh boi cmake/fc_git_version.cmake — KHONG sua tay. */
#ifndef FC_GIT_VERSION_H
#define FC_GIT_VERSION_H
#define FC_GIT_HASH_STR   \"${h16}\"
#define FC_GIT_HASH_BYTES { ${bytes}}
#define FC_GIT_DIRTY      ${dirty}
#endif
")

if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
else()
    set(old "")
endif()

if(NOT old STREQUAL content)
    file(WRITE "${OUT}" "${content}")
endif()
