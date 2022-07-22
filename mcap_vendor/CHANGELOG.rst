^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcap_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2022-07-22)
------------------
* Upgrade mcap to fix LZ4 error and segfault (`#42 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/42>`_)
  Incorporates fixes from https://github.com/foxglove/mcap/pull/478 and https://github.com/foxglove/mcap/pull/482
* Add missing buildtool_depend on git (`#37 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/37>`_)
  This vendor package uses git to fetch sources for other packages. It should declare a dependency on that build tool.
  This should address the current cause of RPM build failures for RHEL: https://build.ros2.org/view/Rbin_rhel_el864/job/Rbin_rhel_el864__mcap_vendor__rhel_8_x86_64__binary/
* Contributors: Jacob Bandes-Storch, Scott K Logan

0.1.5 (2022-04-25)
------------------
* Test Foxy & Galactic in CI, fix missing test_depends in mcap_vendor/package.xml (`#33 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/33>`_)
* Contributors: Jacob Bandes-Storch

0.1.4 (2022-04-21)
------------------
* fix: minor issues (`#31 <https://github.com/wep21/rosbag2_storage_mcap/issues/31>`_)
  * remove unnecessary block
  * use target_link_libraries instead of ament_target_dependencies
  * remove ros environment
  * add prefix to compile definition
* Update email address for Foxglove maintainers (`#32 <https://github.com/wep21/rosbag2_storage_mcap/issues/32>`_)
* Contributors: Daisuke Nishimatsu, Jacob Bandes-Storch

0.1.3 (2022-04-20)
------------------

0.1.2 (2022-04-20)
------------------
* Added mcap_vendor package. Updated CMakeLists.txt to fetch dependencies with FetchContent rather than Conan.
* Contributors: Jacob Bandes-Storch
