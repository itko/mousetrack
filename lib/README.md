# Modules
The collection of header file, source file, and unit testing file, is called a module.
These files usually have the same name and only differ in the suffix.

Header files end with `*.h`. Source files end with `*.cpp`. Unit tests end with `*.test.cc`.

We don't name test files with `cpp`, because, sometimes, it is useful to filter by suffix.
Many environments make it easy to filter by suffix, while it can be more involved to filter by substrings in the filename.

# How to add a module

Header files are added automatically.

Source files need to be added to the variable `source_files` in  [/lib/CMakeLists.txt](CMakeLists.txt).

Unit tests need to be added to the variable `test_files` in [/lib/CMakeLists.txt](CMakeLists.txt).
