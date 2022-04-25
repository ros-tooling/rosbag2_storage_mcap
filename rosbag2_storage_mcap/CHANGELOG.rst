^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_storage_mcap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2022-04-25)
------------------
* Fix build for Foxy (`#34 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/34>`_)
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

0.1.1 (2022-04-01)
------------------
* CMake build script will now execute pip install conan automatically.
* Contributors: Daisuke Nishimatsu

0.1.0 (2022-03-24)
------------------
* [1.0.0] Use Summary section for get_metadata() and seek(), implement remaining methods (`#17 <https://github.com/wep21/rosbag2_storage_mcap/issues/17>`_)
* feat: add play impl (`#16 <https://github.com/wep21/rosbag2_storage_mcap/issues/16>`_)
* chore: refine package.xml (`#15 <https://github.com/wep21/rosbag2_storage_mcap/issues/15>`_)
* Don't throw when READ_WRITE mode is used; add .mcap file extension to recorded files (`#14 <https://github.com/wep21/rosbag2_storage_mcap/issues/14>`_)
  I may be missing something, but from a cursory glance at [this code](https://github.com/ros2/rosbag2/blob/342d8ed3c1c4ae0411a4a92b60e79a728b8974b8/rosbag2_storage/src/rosbag2_storage/impl/storage_factory_impl.hpp#L108-L135), it appears that the `APPEND` mode is never used. This means we need to support `READ_WRITE`.
  This also adds a `.mcap` extension to recorded file names.
* Add dynamic message definition lookup (`#13 <https://github.com/wep21/rosbag2_storage_mcap/issues/13>`_)
  Currently, an exception will be thrown if lookup fails.
* Switch C++ formatter to clang-format (`#12 <https://github.com/wep21/rosbag2_storage_mcap/issues/12>`_)
  Remove uncrustify linter in favor of clang-format, which is easier to configure for use in VS Code format-on-save.
* Merge pull request `#7 <https://github.com/wep21/rosbag2_storage_mcap/issues/7>`_ from ros-tooling/jhurliman/reader-writer
  Reader and writer implementation
* uninitialized struct
* lint
* lint
* lint
* Reader and writer implementation
* Merge pull request `#6 <https://github.com/wep21/rosbag2_storage_mcap/issues/6>`_ from wep21/add-metadata-impl
  feat: add metadata impl
* feat: add metadata impl
* Merge pull request `#5 <https://github.com/wep21/rosbag2_storage_mcap/issues/5>`_ from wep21/mcap-storage-impl
  feat: mcap storage impl
* chore: update cmake minimum version
* chore: install mcap header
* chore: include mcap header
* fix: move fetch content into rosbag2 storage mcap
* Merge pull request `#3 <https://github.com/wep21/rosbag2_storage_mcap/issues/3>`_ from ros-tooling/emersonknapp/mcap_plugin_skeleton
  Add mcap storage plugin skeleton and CI
* Add rosbag2_storage_mcap skeleton
* Contributors: Daisuke Nishimatsu, Emerson Knapp, Jacob Bandes-Storch, John Hurliman, wep21
