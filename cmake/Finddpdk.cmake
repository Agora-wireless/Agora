# https://github.com/tudang/cmake-dpdk/blob/master/cmake/modules/Finddpdk.cmake
# Try to find dpdk
#
# Once done, this will define
#
# dpdk::dpdk
# dpdk_FOUND
# dpdk_INCLUDE_DIRS
# dpdk_LIBRARIES

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(dpdk QUIET libdpdk)
endif()

if(dpdk_INCLUDE_DIRS)
  # good
elseif(TARGET dpdk::dpdk)
  get_target_property(dpdk_INCLUDE_DIRS
     dpdk::dpdk INTERFACE_INCLUDE_DIRECTORIES)
else()
  find_path(dpdk_config_INCLUDE_DIR rte_config.h
    HINTS
       $ENV{RTE_SDK}/$ENV{RTE_TARGET}/include
    PATH_SUFFIXES
      dpdk
      include)
  find_path(dpdk_common_INCLUDE_DIR rte_common.h
    HINTS
      $ENV{RTE_SDK}/$ENV{RTE_TARGET}/include
    PATH_SUFFIXES
      dpdk
      include)
  set(dpdk_INCLUDE_DIRS "${dpdk_config_INCLUDE_DIR}")
  if(NOT dpdk_config_INCLUDE_DIR EQUAL dpdk_common_INCLUDE_DIR)
    list(APPEND dpdk_INCLUDE_DIRS "${dpdk_common_INCLUDE_DIR}")
  endif()
endif()

set(components
  flow_classify
  pipeline
  table
  port
  pdump
  distributor
  ip_frag
  meter
  fib
  rib
  lpm
  ipsec
  cfgfile
  gro
  gso
  hash
  member
  vhost
  kvargs
  mbuf
  net
  ethdev
  bbdev
  cryptodev
  security
  compressdev
  eventdev
  rawdev
  timer
  mempool
  stack
  mempool_ring
  mempool_octeontx2
  ring
  pci
  eal
  cmdline
  reorder
  sched
  rcu
  kni
  common_cpt
  common_octeontx
  common_octeontx2
  common_dpaax
  common_mlx5
  bus_pci
  bus_vdev
  bus_dpaa
  bus_fslmc
  mempool_bucket
  mempool_stack
  mempool_dpaa
  mempool_dpaa2
  pmd_af_packet
  pmd_bnxt
  pmd_bond
  pmd_cxgbe
  pmd_e1000
  pmd_ena
  pmd_enic
  pmd_i40e
  pmd_ixgbe
  pmd_mlx5
  pmd_nfp
  pmd_qede
  pmd_ring
  pmd_sfc_efx
  pmd_vmxnet3_uio
  mempool_octeontx 
  rawdev_skeleton 
  rawdev_dpaa2_cmdif 
  rawdev_dpaa2_qdma 
  bus_ifpga 
  rawdev_ioat 
  rawdev_ntb)


# for collecting dpdk library targets, it will be used when defining dpdk::dpdk
set(_dpdk_libs)
# for list of dpdk library archive paths
set(dpdk_LIBRARIES)

foreach(c ${components})
  set(dpdk_lib dpdk::${c})
  if(TARGET ${dpdk_lib})
    get_target_property(DPDK_rte_${c}_LIBRARY
      ${dpdk_lib} IMPORTED_LOCATION)
  else()
    find_library(DPDK_rte_${c}_LIBRARY rte_${c}
      HINTS
        $ENV{RTE_SDK}/$ENV{RTE_TARGET}/lib
        ${dpdk_LIBRARY_DIRS}
        PATH_SUFFIXES lib)
  endif()
  if(DPDK_rte_${c}_LIBRARY)
    if (NOT TARGET ${dpdk_lib})
      add_library(${dpdk_lib} UNKNOWN IMPORTED)
      set_target_properties(${dpdk_lib} PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${dpdk_INCLUDE_DIRS}"
        IMPORTED_LOCATION "${DPDK_rte_${c}_LIBRARY}")
      if(c STREQUAL pmd_mlx5)
        find_package(verbs QUIET)
        if(verbs_FOUND)
          target_link_libraries(${dpdk_lib} INTERFACE IBVerbs::verbs)
        endif()
      endif()
    endif()
    list(APPEND _dpdk_libs ${dpdk_lib})
    list(APPEND dpdk_LIBRARIES ${DPDK_rte_${c}_LIBRARY})
  endif()
endforeach()

mark_as_advanced(dpdk_INCLUDE_DIRS ${dpdk_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(dpdk DEFAULT_MSG
  dpdk_INCLUDE_DIRS
  dpdk_LIBRARIES)
if(dpdk_FOUND)
  if(NOT TARGET dpdk::cflags)
     if(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64|x86_64|AMD64")
      set(rte_cflags "-march=core2")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM")
      set(rte_cflags "-march=armv7-a")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|AARCH64")
      set(rte_cflags "-march=armv8-a+crc")
    endif()
    add_library(dpdk::cflags INTERFACE IMPORTED)
    if (rte_cflags)
      set_target_properties(dpdk::cflags PROPERTIES
        INTERFACE_COMPILE_OPTIONS "${rte_cflags}")
    endif()
  endif()

  if(NOT TARGET dpdk::dpdk)
    add_library(dpdk::dpdk INTERFACE IMPORTED)
    find_package(Threads QUIET)
    list(APPEND _dpdk_libs
      Threads::Threads
      dpdk::cflags)
    set_target_properties(dpdk::dpdk PROPERTIES
      INTERFACE_LINK_LIBRARIES "${_dpdk_libs}"
      INTERFACE_INCLUDE_DIRECTORIES "${dpdk_INCLUDE_DIRS}")
  endif()

  set(dpdk_LIBRARIES
    ${dpdk_LIBRARIES} -Wl,--whole-archive -libverbs -lmlx5 -Wl,--no-whole-archive -lpthread -lrt -lm -lnuma -ldl)
  # message(STATUS "dpdk_LIBRARIES: ${dpdk_LIBRARIES}")
  set(dpdk_LIBRARIES -lrte_flow_classify -Wl,--whole-archive -lrte_pipeline -Wl,--no-whole-archive -Wl,--whole-archive -lrte_table -Wl,--no-whole-archive -Wl,--whole-archive -lrte_port -Wl,--no-whole-archive -lrte_pdump -lrte_distributor -lrte_ip_frag -lrte_meter -lrte_fib -lrte_rib -lrte_lpm -lrte_acl -lrte_jobstats -lrte_metrics -lrte_bitratestats -lrte_latencystats -lrte_power -lrte_efd -lrte_bpf -lrte_ipsec -Wl,--whole-archive -lrte_cfgfile -lrte_gro -lrte_gso -lrte_hash -lrte_member -lrte_vhost -lrte_kvargs -lrte_mbuf -lrte_net -lrte_ethdev -lrte_bbdev -lrte_cryptodev -lrte_security -lrte_compressdev -lrte_eventdev -lrte_rawdev -lrte_timer -lrte_mempool -lrte_stack -lrte_mempool_ring -lrte_mempool_octeontx2 -lrte_ring -lrte_pci -lrte_eal -lrte_cmdline -lrte_reorder -lrte_sched -lrte_rcu -lrte_kni -lrte_common_cpt -lrte_common_octeontx -lrte_common_octeontx2 -lrte_common_dpaax -lrte_bus_pci -lrte_bus_vdev -lrte_bus_dpaa -lrte_bus_fslmc -lrte_mempool_bucket -lrte_mempool_stack -lrte_mempool_dpaa -lrte_mempool_dpaa2 -lrte_pmd_af_packet -lrte_pmd_ark -lrte_pmd_atlantic -lrte_pmd_avp -lrte_pmd_axgbe -lrte_pmd_bnxt -lrte_pmd_bond -lrte_pmd_cxgbe -lrte_pmd_dpaa -lrte_pmd_dpaa2 -lrte_pmd_e1000 -lrte_pmd_ena -lrte_pmd_enetc -lrte_pmd_enic -lrte_pmd_fm10k -lrte_pmd_failsafe -lrte_pmd_hinic -lrte_pmd_hns3 -lrte_pmd_i40e -lrte_pmd_iavf -lrte_pmd_ice -lrte_common_iavf -lrte_pmd_ionic -lrte_pmd_ixgbe -lrte_pmd_kni -lrte_pmd_lio -lrte_pmd_memif -lrte_common_mlx5 -lrte_pmd_mlx5 -libverbs -lmlx5 -lrte_pmd_nfp -lrte_pmd_null -lrte_pmd_octeontx2 -lrte_pmd_qede -lrte_pmd_ring -lrte_pmd_softnic -lrte_pmd_sfc_efx -lrte_pmd_tap -lrte_pmd_thunderx_nicvf -lrte_pmd_vdev_netvsc -lrte_pmd_virtio -lrte_pmd_vhost -lrte_pmd_ifc -lrte_pmd_vmxnet3_uio -lrte_bus_vmbus -lrte_pmd_netvsc -lrte_pmd_bbdev_null -lrte_pmd_bbdev_fpga_lte_fec -lrte_pmd_bbdev_turbo_sw -lrte_pmd_null_crypto -lrte_pmd_nitrox -lrte_pmd_octeontx_crypto -lrte_pmd_octeontx2_crypto -lrte_pmd_crypto_scheduler -lrte_pmd_dpaa2_sec -lrte_pmd_dpaa_sec -lrte_pmd_caam_jr -lrte_pmd_virtio_crypto -lrte_pmd_octeontx_zip -lrte_pmd_qat -lrte_pmd_skeleton_event -lrte_pmd_sw_event -lrte_pmd_dsw_event -lrte_pmd_octeontx_ssovf -lrte_pmd_dpaa_event -lrte_pmd_dpaa2_event -lrte_mempool_octeontx -lrte_pmd_octeontx -lrte_pmd_octeontx2_event -lrte_pmd_opdl_event -lrte_rawdev_skeleton -lrte_rawdev_dpaa2_cmdif -lrte_rawdev_dpaa2_qdma -lrte_bus_ifpga -lrte_rawdev_ioat -lrte_rawdev_ntb -lrte_rawdev_octeontx2_dma -lrte_rawdev_octeontx2_ep -Wl,--no-whole-archive -lpthread -lrt -lm -lnuma -ldl)

endif()

unset(_dpdk_libs)