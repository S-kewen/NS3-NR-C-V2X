#! /usr/bin/env python3
# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-
#
# Copyright (c) 2021 Universidade de Brasília
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Author: Gabriel Ferreira <gabrielcarvfer@gmail.com>

"""!
Test suite for the ns3 wrapper script
"""

import glob
import os
import re
import shutil
import subprocess
import sys
import unittest
from functools import partial

# Get path containing ns3
ns3_path = os.path.dirname(os.path.abspath(os.sep.join([__file__, "../../"])))
ns3_lock_filename = os.path.join(ns3_path, ".lock-ns3_%s_build" % sys.platform)
ns3_script = os.sep.join([ns3_path, "ns3"])
ns3rc_script = os.sep.join([ns3_path, ".ns3rc"])
usual_outdir = os.sep.join([ns3_path, "build"])
usual_lib_outdir = os.sep.join([usual_outdir, "lib"])

# Move the current working directory to the ns-3-dev folder
os.chdir(ns3_path)

# Cmake commands
num_threads = max(1, os.cpu_count() - 1)
cmake_build_project_command = "cmake --build . -j".format(ns3_path=ns3_path)
cmake_build_target_command = partial("cmake --build . -j {jobs} --target {target}".format,
                                     jobs=num_threads
                                     )


def run_ns3(args, env=None):
    """!
    Runs the ns3 wrapper script with arguments
    @param args: string containing arguments that will get split before calling ns3
    @param env: environment variables dictionary
    @return tuple containing (error code, stdout and stderr)
    """
    if "clean" in args:
        possible_leftovers = ["contrib/borked", "contrib/calibre"]
        for leftover in possible_leftovers:
            if os.path.exists(leftover):
                shutil.rmtree(leftover, ignore_errors=True)
    return run_program(ns3_script, args, python=True, env=env)


# Adapted from https://github.com/metabrainz/picard/blob/master/picard/util/__init__.py
def run_program(program, args, python=False, cwd=ns3_path, env=None):
    """!
    Runs a program with the given arguments and returns a tuple containing (error code, stdout and stderr)
    @param program: program to execute (or python script)
    @param args: string containing arguments that will get split before calling the program
    @param python: flag indicating whether the program is a python script
    @param cwd: the working directory used that will be the root folder for the execution
    @param env: environment variables dictionary
    @return tuple containing (error code, stdout and stderr)
    """
    if type(args) != str:
        raise Exception("args should be a string")

    # Include python interpreter if running a python script
    if python:
        arguments = [sys.executable, program]
    else:
        arguments = [program]

    if args != "":
        arguments.extend(re.findall("(?:\".*?\"|\S)+", args))  # noqa

    for i in range(len(arguments)):
        arguments[i] = arguments[i].replace("\"", "")

    # Forward environment variables used by the ns3 script
    current_env = os.environ.copy()

    # Add different environment variables
    if env:
        current_env.update(env)

    # Call program with arguments
    ret = subprocess.run(
        arguments,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=cwd,  # run process from the ns-3-dev path
        env=current_env
    )
    # Return (error code, stdout and stderr)
    return ret.returncode, ret.stdout.decode(sys.stdout.encoding), ret.stderr.decode(sys.stderr.encoding)


def get_programs_list():
    """!
    Extracts the programs list from .lock-ns3
    @return list of programs.
    """
    values = {}
    with open(ns3_lock_filename) as f:
        exec(f.read(), globals(), values)
    return values["ns3_runnable_programs"]


def get_libraries_list(lib_outdir=usual_lib_outdir):
    """!
    Gets a list of built libraries
    @param lib_outdir: path containing libraries
    @return list of built libraries.
    """
    return glob.glob(lib_outdir + '/*', recursive=True)


def get_headers_list(outdir=usual_outdir):
    """!
    Gets a list of header files
    @param outdir: path containing headers
    @return list of headers.
    """
    return glob.glob(outdir + '/**/*.h', recursive=True)


def read_lock_entry(entry):
    """!
    Read interesting entries from the .lock-ns3 file
    @param entry: entry to read from .lock-ns3
    @return value of the requested entry.
    """
    values = {}
    with open(ns3_lock_filename) as f:
        exec(f.read(), globals(), values)
    return values.get(entry, None)


def get_test_enabled():
    """!
    Check if tests are enabled in the .lock-ns3
    @return bool.
    """
    return read_lock_entry("ENABLE_TESTS")


def get_enabled_modules():
    """
    Check if tests are enabled in the .lock-ns3
    @return list of enabled modules (prefixed with 'ns3-').
    """
    return read_lock_entry("NS3_ENABLED_MODULES")


class NS3UnusedSourcesTestCase(unittest.TestCase):
    """!
    ns-3 tests related to checking if source files were left behind, not being used by CMake
    """

    ## dictionary containing directories with .cc source files # noqa
    directory_and_files = {}

    def setUp(self):
        """!
        Scan all C++ source files and add them to a list based on their path
        @return None
        """
        for root, dirs, files in os.walk(ns3_path):
            if "gitlab-ci-local" in root:
                continue
            for name in files:
                if name.endswith(".cc"):
                    path = os.path.join(root, name)
                    directory = os.path.dirname(path)
                    if directory not in self.directory_and_files:
                        self.directory_and_files[directory] = []
                    self.directory_and_files[directory].append(path)

    def test_01_UnusedExampleSources(self):
        """!
        Test if all example source files are being used in their respective CMakeLists.txt
        @return None
        """
        unused_sources = set()
        for example_directory in self.directory_and_files.keys():
            # Skip non-example directories
            if os.sep + "examples" not in example_directory:
                continue

            # Open the examples CMakeLists.txt and read it
            with open(os.path.join(example_directory, "CMakeLists.txt"), "r") as f:
                cmake_contents = f.read()

            # For each file, check if it is in the CMake contents
            for file in self.directory_and_files[example_directory]:
                # We remove the .cc because some examples sources can be written as ${example_name}.cc
                if os.path.basename(file).replace(".cc", "") not in cmake_contents:
                    unused_sources.add(file)

        self.assertListEqual([], list(unused_sources))

    def test_02_UnusedModuleSources(self):
        """!
        Test if all module source files are being used in their respective CMakeLists.txt
        @return None
        """
        unused_sources = set()
        for directory in self.directory_and_files.keys():
            # Skip examples and bindings directories
            is_not_module = not ("src" in directory or "contrib" in directory)
            is_example = os.sep + "examples" in directory
            is_bindings = os.sep + "bindings" in directory

            if is_not_module or is_bindings or is_example:
                continue

            # We can be in one of the module subdirectories (helper, model, test, bindings, etc)
            # Navigate upwards until we hit a CMakeLists.txt
            cmake_path = os.path.join(directory, "CMakeLists.txt")
            while not os.path.exists(cmake_path):
                parent_directory = os.path.dirname(os.path.dirname(cmake_path))
                cmake_path = os.path.join(parent_directory, os.path.basename(cmake_path))

            # Open the module CMakeLists.txt and read it
            with open(cmake_path, "r") as f:
                cmake_contents = f.read()

            # For each file, check if it is in the CMake contents
            for file in self.directory_and_files[directory]:
                if os.path.basename(file) not in cmake_contents:
                    unused_sources.add(file)

        # Remove temporary exceptions
        exceptions = ["win32-system-wall-clock-ms.cc",  # Should be removed with MR784
                      ]
        for exception in exceptions:
            for unused_source in unused_sources:
                if os.path.basename(unused_source) == exception:
                    unused_sources.remove(unused_source)
                    break

        self.assertListEqual([], list(unused_sources))

    def test_03_UnusedUtilsSources(self):
        """!
        Test if all utils source files are being used in their respective CMakeLists.txt
        @return None
        """
        unused_sources = set()
        for directory in self.directory_and_files.keys():
            # Skip directories that are not utils
            is_module = "src" in directory or "contrib" in directory
            if os.sep + "utils" not in directory or is_module:
                continue

            # We can be in one of the module subdirectories (helper, model, test, bindings, etc)
            # Navigate upwards until we hit a CMakeLists.txt
            cmake_path = os.path.join(directory, "CMakeLists.txt")
            while not os.path.exists(cmake_path):
                parent_directory = os.path.dirname(os.path.dirname(cmake_path))
                cmake_path = os.path.join(parent_directory, os.path.basename(cmake_path))

            # Open the module CMakeLists.txt and read it
            with open(cmake_path, "r") as f:
                cmake_contents = f.read()

            # For each file, check if it is in the CMake contents
            for file in self.directory_and_files[directory]:
                if os.path.basename(file) not in cmake_contents:
                    unused_sources.add(file)

        self.assertListEqual([], list(unused_sources))


class NS3DependenciesTestCase(unittest.TestCase):
    """!
    ns-3 tests related to dependencies
    """

    def test_01_CheckIfIncludedHeadersMatchLinkedModules(self):
        """!
        Checks if headers from different modules (src/A, contrib/B) that are included by
        the current module (src/C) source files correspond to the list of linked modules
        LIBNAME C
        LIBRARIES_TO_LINK A (missing B)
        @return None
        """
        modules = {}
        headers_to_modules = {}
        module_paths = glob.glob(ns3_path + "/src/*/") + glob.glob(ns3_path + "/contrib/*/")

        for path in module_paths:
            # Open the module CMakeLists.txt and read it
            cmake_path = os.path.join(path, "CMakeLists.txt")
            with open(cmake_path, "r") as f:
                cmake_contents = f.readlines()

            module_name = os.path.relpath(path, ns3_path)
            module_name_nodir = module_name.replace("src/", "").replace("contrib/", "")
            modules[module_name_nodir] = {"sources": set(),
                                          "headers": set(),
                                          "libraries": set(),
                                          "included_headers": set(),
                                          "included_libraries": set(),
                                          }

            # Separate list of source files and header files
            for line in cmake_contents:
                base_name = os.path.basename(line[:-1])
                if not os.path.exists(os.path.join(path, line.strip())):
                    continue

                if ".h" in line:
                    # Register all module headers as module headers and sources
                    modules[module_name_nodir]["headers"].add(base_name)
                    modules[module_name_nodir]["sources"].add(base_name)

                    # Register the header as part of the current module
                    headers_to_modules[base_name] = module_name_nodir

                if ".cc" in line:
                    # Register the source file as part of the current module
                    modules[module_name_nodir]["sources"].add(base_name)

                if ".cc" in line or ".h" in line:
                    # Extract includes from headers and source files and then add to a list of included headers
                    source_file = os.path.join(ns3_path, module_name, line.strip())
                    with open(source_file, "r", encoding="utf-8") as f:
                        source_contents = f.read()
                    modules[module_name_nodir]["included_headers"].update(map(lambda x: x.replace("ns3/", ""),
                                                                              re.findall("#include.*[\"|<](.*)[\"|>]",
                                                                                         source_contents)
                                                                              )
                                                                          )
                    continue

            # Extract libraries linked to the module
            modules[module_name_nodir]["libraries"].update(re.findall("\\${lib(.*)}", "".join(cmake_contents)))

        # Now that we have all the information we need, check if we have all the included libraries linked
        all_project_headers = set(headers_to_modules.keys())

        sys.stderr.flush()
        print(file=sys.stderr)
        for module in sorted(modules):
            external_headers = modules[module]["included_headers"].difference(all_project_headers)
            project_headers_included = modules[module]["included_headers"].difference(external_headers)
            modules[module]["included_libraries"] = set(
                [headers_to_modules[x] for x in project_headers_included]).difference(
                {module})

            diff = modules[module]["included_libraries"].difference(modules[module]["libraries"])
            if len(diff) > 0:
                print("Module %s includes modules that are not linked: %s" % (module, ", ".join(list(diff))),
                      file=sys.stderr)
                sys.stderr.flush()
            # Uncomment this to turn into a real test
            # self.assertEqual(len(diff), 0,
            #                 msg="Module %s includes modules that are not linked: %s" % (module, ", ".join(list(diff)))
            #                 )
        self.assertTrue(True)


class NS3StyleTestCase(unittest.TestCase):
    """!
    ns-3 tests to check if the source code, whitespaces and CMake formatting
    are according to the coding style
    """

    ## Holds the original diff, which must be maintained by each and every case tested # noqa
    starting_diff = None
    ## Holds the GitRepo's repository object # noqa
    repo = None

    def setUp(self) -> None:
        """!
        Import GitRepo and load the original diff state of the repository before the tests
        @return None
        """
        if not NS3StyleTestCase.starting_diff:

            if shutil.which("git") is None:
                self.skipTest("Git is not available")

            try:
                from git import Repo
                import git.exc
            except ImportError:
                self.skipTest("GitPython is not available")

            try:
                repo = Repo(ns3_path)  # noqa
            except git.exc.InvalidGitRepositoryError:  # noqa
                self.skipTest("ns-3 directory does not contain a .git directory")

            hcommit = repo.head.commit  # noqa
            NS3StyleTestCase.starting_diff = hcommit.diff(None)
            NS3StyleTestCase.repo = repo

        if NS3StyleTestCase.starting_diff is None:
            self.skipTest("Unmet dependencies")

    def test_01_CheckWhitespace(self):
        """!
        Check if there is any difference between tracked file after
        applying whitespace trimming, uncrustify and cmake-format
        @return None
        """

        # Trim all trailing whitespaces in the repository
        return_code, stdout, stderr = run_program("./utils/trim-trailing-whitespace.py", "./", python=True)
        self.assertEqual(return_code, 0)

        # Check if the diff still is the same
        new_diff = NS3StyleTestCase.repo.head.commit.diff(None)
        self.assertEqual(NS3StyleTestCase.starting_diff, new_diff)

    def test_02_CheckUncrustify(self):
        """!
        Check if there is any difference between tracked file after
        applying uncrustify
        @return None
        """

        for required_program in ["bash", "find", "xargs", "uncrustify"]:
            if shutil.which(required_program) is None:
                self.skipTest("%s was not found" % required_program)

        # Run in-place uncrustify for each *.cc and *.h file
        uncrustify_cmd = """-c "find . -type f -name '*.[cc|h]' | xargs -I{} ./utils/check-style.py -i -f {}" """
        return_code, stdout, stderr = run_program("bash", uncrustify_cmd)
        self.assertEqual(return_code, 0)

        # Check if the diff still is the same
        new_diff = NS3StyleTestCase.repo.head.commit.diff(None)
        self.assertEqual(NS3StyleTestCase.starting_diff, new_diff)

    def test_03_CheckCMakeFormat(self):
        """!
        Check if there is any difference between tracked file after
        applying cmake-format
        @return None
        """

        for required_program in ["cmake", "cmake-format"]:
            if shutil.which(required_program) is None:
                self.skipTest("%s was not found" % required_program)

        # Configure ns-3 to get the cmake-format target
        return_code, stdout, stderr = run_ns3("configure")
        self.assertEqual(return_code, 0)

        # Build/run cmake-format
        return_code, stdout, stderr = run_ns3("build cmake-format")
        self.assertEqual(return_code, 0)

        # Clean the ns-3 configuration
        return_code, stdout, stderr = run_ns3("clean")
        self.assertEqual(return_code, 0)

        # Check if the diff still is the same
        new_diff = NS3StyleTestCase.repo.head.commit.diff(None)
        self.assertEqual(NS3StyleTestCase.starting_diff, new_diff)


class NS3CommonSettingsTestCase(unittest.TestCase):
    """!
    ns3 tests related to generic options
    """

    def setUp(self):
        """!
        Clean configuration/build artifacts before common commands
        @return None
        """
        super().setUp()
        # No special setup for common test cases other than making sure we are working on a clean directory.
        run_ns3("clean")

    def test_01_NoOption(self):
        """!
        Test not passing any arguments to
        @return None
        """
        return_code, stdout, stderr = run_ns3("")
        self.assertEqual(return_code, 1)
        self.assertIn("You need to configure ns-3 first: try ./ns3 configure", stdout)

    def test_02_NoTaskLines(self):
        """!
        Test only passing --quiet argument to ns3
        @return None
        """
        return_code, stdout, stderr = run_ns3("--quiet")
        self.assertEqual(return_code, 1)
        self.assertIn("You need to configure ns-3 first: try ./ns3 configure", stdout)

    def test_03_CheckConfig(self):
        """!
        Test only passing 'show config' argument to ns3
        @return None
        """
        return_code, stdout, stderr = run_ns3("show config")
        self.assertEqual(return_code, 1)
        self.assertIn("You need to configure ns-3 first: try ./ns3 configure", stdout)

    def test_04_CheckProfile(self):
        """!
        Test only passing 'show profile' argument to ns3
        @return None
        """
        return_code, stdout, stderr = run_ns3("show profile")
        self.assertEqual(return_code, 1)
        self.assertIn("You need to configure ns-3 first: try ./ns3 configure", stdout)

    def test_05_CheckVersion(self):
        """!
        Test only passing 'show version' argument to ns3
        @return None
        """
        return_code, stdout, stderr = run_ns3("show version")
        self.assertEqual(return_code, 1)
        self.assertIn("You need to configure ns-3 first: try ./ns3 configure", stdout)


class NS3ConfigureBuildProfileTestCase(unittest.TestCase):
    """!
    ns3 tests related to build profiles
    """

    def setUp(self):
        """!
        Clean configuration/build artifacts before testing configuration settings
        @return None
        """
        super().setUp()
        # No special setup for common test cases other than making sure we are working on a clean directory.
        run_ns3("clean")

    def test_01_Debug(self):
        """!
        Test the debug build
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" -d debug --enable-verbose")
        self.assertEqual(return_code, 0)
        self.assertIn("Build profile                 : debug", stdout)
        self.assertIn("Build files have been written to", stdout)

        # Build core to check if profile suffixes match the expected.
        return_code, stdout, stderr = run_ns3("build core")
        self.assertEqual(return_code, 0)
        self.assertIn("Built target libcore", stdout)

        libraries = get_libraries_list()
        self.assertGreater(len(libraries), 0)
        self.assertIn("core-debug", libraries[0])

    def test_02_Release(self):
        """!
        Test the release build
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" -d release")
        self.assertEqual(return_code, 0)
        self.assertIn("Build profile                 : release", stdout)
        self.assertIn("Build files have been written to", stdout)

    def test_03_Optimized(self):
        """!
        Test the optimized build
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" -d optimized --enable-verbose")
        self.assertEqual(return_code, 0)
        self.assertIn("Build profile                 : optimized", stdout)
        self.assertIn("Build files have been written to", stdout)

        # Build core to check if profile suffixes match the expected
        return_code, stdout, stderr = run_ns3("build core")
        self.assertEqual(return_code, 0)
        self.assertIn("Built target libcore", stdout)

        libraries = get_libraries_list()
        self.assertGreater(len(libraries), 0)
        self.assertIn("core-optimized", libraries[0])

    def test_04_Typo(self):
        """!
        Test a build type with a typo
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" -d Optimized")
        self.assertEqual(return_code, 2)
        self.assertIn("invalid choice: 'Optimized'", stderr)

    def test_05_TYPO(self):
        """!
        Test a build type with another typo
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" -d OPTIMIZED")
        self.assertEqual(return_code, 2)
        self.assertIn("invalid choice: 'OPTIMIZED'", stderr)


class NS3BaseTestCase(unittest.TestCase):
    """!
    Generic test case with basic function inherited by more complex tests.
    """

    ## when cleaned_once is False, clean up build artifacts and reconfigure # noqa
    cleaned_once = False

    def config_ok(self, return_code, stdout):
        """!
        Check if configuration for release mode worked normally
        @param return_code: return code from CMake
        @param stdout: output from CMake.
        @return None
        """
        self.assertEqual(return_code, 0)
        self.assertIn("Build profile                 : release", stdout)
        self.assertIn("Build files have been written to", stdout)

    def setUp(self):
        """!
        Clean configuration/build artifacts before testing configuration and build settings
        After configuring the build as release,
        check if configuration worked and check expected output files.
        @return None
        """
        super().setUp()

        if os.path.exists(ns3rc_script):
            os.remove(ns3rc_script)

        # We only clear it once and then update the settings by changing flags or consuming ns3rc.
        if not NS3BaseTestCase.cleaned_once:
            NS3BaseTestCase.cleaned_once = True
            run_ns3("clean")
            return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" -d release --enable-verbose")
            self.config_ok(return_code, stdout)

        # Check if .lock-ns3 exists, then read to get list of executables.
        self.assertTrue(os.path.exists(ns3_lock_filename))
        ## ns3_executables holds a list of executables in .lock-ns3 # noqa
        self.ns3_executables = get_programs_list()

        # Check if .lock-ns3 exists than read to get the list of enabled modules.
        self.assertTrue(os.path.exists(ns3_lock_filename))
        ## ns3_modules holds a list to the modules enabled stored in .lock-ns3 # noqa
        self.ns3_modules = get_enabled_modules()


class NS3ConfigureTestCase(NS3BaseTestCase):
    """!
    Test ns3 configuration options
    """

    ## when cleaned_once is False, clean up build artifacts and reconfigure # noqa
    cleaned_once = False

    def setUp(self):
        """!
        Reuse cleaning/release configuration from NS3BaseTestCase if flag is cleaned
        @return None
        """
        if not NS3ConfigureTestCase.cleaned_once:
            NS3ConfigureTestCase.cleaned_once = True
            NS3BaseTestCase.cleaned_once = False
        super().setUp()

    def test_01_Examples(self):
        """!
        Test enabling and disabling examples
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-examples")

        # This just tests if we didn't break anything, not that we actually have enabled anything.
        self.config_ok(return_code, stdout)

        # If nothing went wrong, we should have more executables in the list after enabling the examples.
        self.assertGreater(len(get_programs_list()), len(self.ns3_executables))

        # Now we disabled them back.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --disable-examples")

        # This just tests if we didn't break anything, not that we actually have enabled anything.
        self.config_ok(return_code, stdout)

        # Then check if they went back to the original list.
        self.assertEqual(len(get_programs_list()), len(self.ns3_executables))

    def test_02_Tests(self):
        """!
        Test enabling and disabling tests
        @return None
        """
        # Try enabling tests
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-tests")
        self.config_ok(return_code, stdout)

        # Then try building the libcore test
        return_code, stdout, stderr = run_ns3("build core-test")

        # If nothing went wrong, this should have worked
        self.assertEqual(return_code, 0)
        self.assertIn("Built target libcore-test", stdout)

        # Now we disabled the tests
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --disable-tests")
        self.config_ok(return_code, stdout)

        # Now building the library test should fail
        return_code, stdout, stderr = run_ns3("build core-test")

        # Then check if they went back to the original list
        self.assertEqual(return_code, 1)
        self.assertIn("Target to build does not exist: core-test", stdout)

    def test_03_EnableModules(self):
        """!
        Test enabling specific modules
        @return None
        """
        # Try filtering enabled modules to network+Wi-Fi and their dependencies
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-modules='network;wifi'")
        self.config_ok(return_code, stdout)

        # At this point we should have fewer modules
        enabled_modules = get_enabled_modules()
        self.assertLess(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertIn("ns3-network", enabled_modules)
        self.assertIn("ns3-wifi", enabled_modules)

        # Try enabling only core
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-modules='core'")
        self.config_ok(return_code, stdout)
        self.assertIn("ns3-core", get_enabled_modules())

        # Try cleaning the list of enabled modules to reset to the normal configuration.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-modules=''")
        self.config_ok(return_code, stdout)

        # At this point we should have the same amount of modules that we had when we started.
        self.assertEqual(len(get_enabled_modules()), len(self.ns3_modules))

    def test_04_DisableModules(self):
        """!
        Test disabling specific modules
        @return None
        """
        # Try filtering disabled modules to disable lte and modules that depend on it.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --disable-modules='lte;wimax'")
        self.config_ok(return_code, stdout)

        # At this point we should have fewer modules.
        enabled_modules = get_enabled_modules()
        self.assertLess(len(enabled_modules), len(self.ns3_modules))
        self.assertNotIn("ns3-lte", enabled_modules)
        self.assertNotIn("ns3-wimax", enabled_modules)

        # Try cleaning the list of enabled modules to reset to the normal configuration.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --disable-modules=''")
        self.config_ok(return_code, stdout)

        # At this point we should have the same amount of modules that we had when we started.
        self.assertEqual(len(get_enabled_modules()), len(self.ns3_modules))

    def test_05_EnableModulesComma(self):
        """!
        Test enabling comma-separated (waf-style) examples
        @return None
        """
        # Try filtering enabled modules to network+Wi-Fi and their dependencies.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-modules='network,wifi'")
        self.config_ok(return_code, stdout)

        # At this point we should have fewer modules.
        enabled_modules = get_enabled_modules()
        self.assertLess(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertIn("ns3-network", enabled_modules)
        self.assertIn("ns3-wifi", enabled_modules)

        # Try cleaning the list of enabled modules to reset to the normal configuration.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-modules=''")
        self.config_ok(return_code, stdout)

        # At this point we should have the same amount of modules that we had when we started.
        self.assertEqual(len(get_enabled_modules()), len(self.ns3_modules))

    def test_06_DisableModulesComma(self):
        """!
        Test disabling comma-separated (waf-style) examples
        @return None
        """
        # Try filtering disabled modules to disable lte and modules that depend on it.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --disable-modules='lte,mpi'")
        self.config_ok(return_code, stdout)

        # At this point we should have fewer modules.
        enabled_modules = get_enabled_modules()
        self.assertLess(len(enabled_modules), len(self.ns3_modules))
        self.assertNotIn("ns3-lte", enabled_modules)
        self.assertNotIn("ns3-mpi", enabled_modules)

        # Try cleaning the list of enabled modules to reset to the normal configuration.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --disable-modules=''")
        self.config_ok(return_code, stdout)

        # At this point we should have the same amount of modules that we had when we started.
        self.assertEqual(len(get_enabled_modules()), len(self.ns3_modules))

    def test_07_Ns3rc(self):
        """!
        Test loading settings from the ns3rc config file
        @return None
        """
        ns3rc_template = "# ! /usr/bin/env python\
                          \
                          # A list of the modules that will be enabled when ns-3 is run.\
                          # Modules that depend on the listed modules will be enabled also.\
                          #\
                          # All modules can be enabled by choosing 'all_modules'.\
                          modules_enabled = [{modules}]\
                          \
                          # Set this equal to true if you want examples to be run.\
                          examples_enabled = {examples}\
                          \
                          # Set this equal to true if you want tests to be run.\
                          tests_enabled = {tests}\
                          "

        # Now we repeat the command line tests but with the ns3rc file.
        with open(ns3rc_script, "w") as f:
            f.write(ns3rc_template.format(modules="'lte'", examples="False", tests="True"))

        # Reconfigure.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.config_ok(return_code, stdout)

        # Check.
        enabled_modules = get_enabled_modules()
        self.assertLess(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertIn("ns3-lte", enabled_modules)
        self.assertTrue(get_test_enabled())
        self.assertEqual(len(get_programs_list()), len(self.ns3_executables))

        # Replace the ns3rc file with the wifi module, enabling examples and disabling tests
        with open(ns3rc_script, "w") as f:
            f.write(ns3rc_template.format(modules="'wifi'", examples="True", tests="False"))

        # Reconfigure
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.config_ok(return_code, stdout)

        # Check
        enabled_modules = get_enabled_modules()
        self.assertLess(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertIn("ns3-wifi", enabled_modules)
        self.assertFalse(get_test_enabled())
        self.assertGreater(len(get_programs_list()), len(self.ns3_executables))

        # Replace the ns3rc file with multiple modules
        with open(ns3rc_script, "w") as f:
            f.write(ns3rc_template.format(modules="'core','network'", examples="True", tests="False"))

        # Reconfigure
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.config_ok(return_code, stdout)

        # Check
        enabled_modules = get_enabled_modules()
        self.assertLess(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertIn("ns3-core", enabled_modules)
        self.assertIn("ns3-network", enabled_modules)
        self.assertFalse(get_test_enabled())
        self.assertGreater(len(get_programs_list()), len(self.ns3_executables))

        # Replace the ns3rc file with multiple modules,
        # in various different ways and with comments
        with open(ns3rc_script, "w") as f:
            f.write(ns3rc_template.format(modules="""'core', #comment
            'lte',
            #comment2,
            #comment3
            'network', 'internet','wimax'""", examples="True", tests="True"))

        # Reconfigure
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.config_ok(return_code, stdout)

        # Check
        enabled_modules = get_enabled_modules()
        self.assertLess(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertIn("ns3-core", enabled_modules)
        self.assertIn("ns3-internet", enabled_modules)
        self.assertIn("ns3-lte", enabled_modules)
        self.assertIn("ns3-wimax", enabled_modules)
        self.assertTrue(get_test_enabled())
        self.assertGreater(len(get_programs_list()), len(self.ns3_executables))

        # Then we roll back by removing the ns3rc config file
        os.remove(ns3rc_script)

        # Reconfigure
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.config_ok(return_code, stdout)

        # Check
        self.assertEqual(len(get_enabled_modules()), len(self.ns3_modules))
        self.assertFalse(get_test_enabled())
        self.assertEqual(len(get_programs_list()), len(self.ns3_executables))

    def test_08_DryRun(self):
        """!
        Test dry-run (printing commands to be executed instead of running them)
        @return None
        """
        run_ns3("clean")

        # Try dry-run before and after the positional commands (outputs should match)
        for positional_command in ["configure", "build", "clean"]:
            return_code, stdout, stderr = run_ns3("--dry-run %s" % positional_command)
            return_code1, stdout1, stderr1 = run_ns3("%s --dry-run" % positional_command)

            self.assertEqual(return_code, return_code1)
            self.assertEqual(stdout, stdout1)
            self.assertEqual(stderr, stderr1)

        run_ns3("clean")

        # Build target before using below
        run_ns3("configure -G \"Unix Makefiles\" -d release --enable-verbose")
        run_ns3("build scratch-simulator")

        # Run all cases and then check outputs
        return_code0, stdout0, stderr0 = run_ns3("--dry-run run scratch-simulator")
        return_code1, stdout1, stderr1 = run_ns3("run scratch-simulator")
        return_code2, stdout2, stderr2 = run_ns3("--dry-run run scratch-simulator --no-build")
        return_code3, stdout3, stderr3 = run_ns3("run scratch-simulator --no-build")

        # Return code and stderr should be the same for all of them.
        self.assertEqual(sum([return_code0, return_code1, return_code2, return_code3]), 0)
        self.assertEqual([stderr0, stderr1, stderr2, stderr3], [""] * 4)

        scratch_path = None
        for program in get_programs_list():
            if "scratch-simulator" in program and "subdir" not in program:
                scratch_path = program
                break

        # Scratches currently have a 'scratch_' prefix in their CMake targets
        # Case 0: dry-run + run (should print commands to build target and then run)
        self.assertIn(cmake_build_target_command(target="scratch_scratch-simulator"), stdout0)
        self.assertIn(scratch_path, stdout0)

        # Case 1: run (should print only make build message)
        self.assertNotIn(cmake_build_target_command(target="scratch_scratch-simulator"), stdout1)
        self.assertIn("Built target", stdout1)
        self.assertNotIn(scratch_path, stdout1)

        # Case 2: dry-run + run-no-build (should print commands to run the target)
        self.assertIn("The following commands would be executed:", stdout2)
        self.assertIn(scratch_path, stdout2)

        # Case 3: run-no-build (should print the target output only)
        self.assertNotIn("Finished executing the following commands:", stdout3)
        self.assertNotIn(scratch_path, stdout3)

    def test_09_PropagationOfReturnCode(self):
        """!
        Test if ns3 is propagating back the return code from the executables called with the run command
        @return None
        """
        # From this point forward we are reconfiguring in debug mode
        return_code, _, _ = run_ns3("clean")
        self.assertEqual(return_code, 0)

        return_code, _, _ = run_ns3("configure -G \"Unix Makefiles\" --enable-examples --enable-tests")
        self.assertEqual(return_code, 0)

        # Build necessary executables
        return_code, stdout, stderr = run_ns3("build command-line-example test-runner")
        self.assertEqual(return_code, 0)

        # Now some tests will succeed normally
        return_code, stdout, stderr = run_ns3("run \"test-runner --test-name=command-line\" --no-build")
        self.assertEqual(return_code, 0)

        # Now some tests will fail during NS_COMMANDLINE_INTROSPECTION
        return_code, stdout, stderr = run_ns3("run \"test-runner --test-name=command-line\" --no-build",
                                              env={"NS_COMMANDLINE_INTROSPECTION": ".."}
                                              )
        self.assertNotEqual(return_code, 0)

        # Cause a sigsegv
        sigsegv_example = os.path.join(ns3_path, "scratch", "sigsegv.cc")
        with open(sigsegv_example, "w") as f:
            f.write("""
                    int main (int argc, char *argv[])
                    {
                      char *s = "hello world"; *s = 'H';
                      return 0;
                    }
                    """)
        return_code, stdout, stderr = run_ns3("run sigsegv")
        self.assertEqual(return_code, 245)
        self.assertIn("sigsegv-default' died with <Signals.SIGSEGV: 11>", stdout)

        # Cause an abort
        abort_example = os.path.join(ns3_path, "scratch", "abort.cc")
        with open(abort_example, "w") as f:
            f.write("""
                    #include "ns3/core-module.h"

                    using namespace ns3;
                    int main (int argc, char *argv[])
                    {
                      NS_ABORT_IF(true);
                      return 0;
                    }
                    """)
        return_code, stdout, stderr = run_ns3("run abort")
        self.assertEqual(return_code, 250)
        self.assertIn("abort-default' died with <Signals.SIGABRT: 6>", stdout)

        os.remove(sigsegv_example)
        os.remove(abort_example)

    def test_10_CheckConfig(self):
        """!
        Test passing 'show config' argument to ns3 to get the configuration table
        @return None
        """
        return_code, stdout, stderr = run_ns3("show config")
        self.assertEqual(return_code, 0)
        self.assertIn("Summary of optional ns-3 features", stdout)

    def test_11_CheckProfile(self):
        """!
        Test passing 'show profile' argument to ns3 to get the build profile
        @return None
        """
        return_code, stdout, stderr = run_ns3("show profile")
        self.assertEqual(return_code, 0)
        self.assertIn("Build profile: default", stdout)

    def test_12_CheckVersion(self):
        """!
        Test passing 'show version' argument to ns3 to get the build version
        @return None
        """
        if shutil.which("git") is None:
            self.skipTest("git is not available")

        return_code, _, _ = run_ns3("configure -G \"Unix Makefiles\" --enable-build-version")
        self.assertEqual(return_code, 0)

        return_code, stdout, stderr = run_ns3("show version")
        self.assertEqual(return_code, 0)
        self.assertIn("ns-3 version:", stdout)

    def test_13_Scratches(self):
        """!
        Test if CMake target names for scratches and ns3 shortcuts
        are working correctly
        @return None
        """

        test_files = ["scratch/main.cc",
                      "scratch/empty.cc",
                      "scratch/subdir1/main.cc",
                      "scratch/subdir2/main.cc"]
        backup_files = ["scratch/.main.cc"]  # hidden files should be ignored

        # Create test scratch files
        for path in test_files + backup_files:
            filepath = os.path.join(ns3_path, path)
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, "w") as f:
                if "main" in path:
                    f.write("int main (int argc, char *argv[]){}")
                else:
                    # no main function will prevent this target from
                    # being created, we should skip it and continue
                    # processing without crashing
                    f.write("")

        # Reload the cmake cache to pick them up
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.assertEqual(return_code, 0)

        # Try to build them with ns3 and cmake
        for path in test_files + backup_files:
            path = path.replace(".cc", "")
            return_code1, stdout1, stderr1 = run_program("cmake", "--build . --target %s -j %d"
                                                         % (path.replace("/", "_"), num_threads),
                                                         cwd=os.path.join(ns3_path, "cmake-cache"))
            return_code2, stdout2, stderr2 = run_ns3("build %s" % path)
            if "main" in path and ".main" not in path:
                self.assertEqual(return_code1, 0)
                self.assertEqual(return_code2, 0)
            else:
                self.assertEqual(return_code1, 2)
                self.assertEqual(return_code2, 1)

        # Try to run them
        for path in test_files:
            path = path.replace(".cc", "")
            return_code, stdout, stderr = run_ns3("run %s --no-build" % path)
            if "main" in path:
                self.assertEqual(return_code, 0)
            else:
                self.assertEqual(return_code, 1)

        # Delete the test files and reconfigure to clean them up
        for path in test_files + backup_files:
            source_absolute_path = os.path.join(ns3_path, path)
            os.remove(source_absolute_path)
            if "empty" in path or ".main" in path:
                continue
            filename = os.path.basename(path).replace(".cc", "")
            executable_absolute_path = os.path.dirname(os.path.join(ns3_path, "build", path))
            executable_name = list(filter(lambda x: filename in x,
                                          os.listdir(executable_absolute_path)
                                          )
                                   )[0]

            os.remove(os.path.join(executable_absolute_path, executable_name))
            if path not in ["scratch/main.cc", "scratch/empty.cc"]:
                os.rmdir(os.path.dirname(source_absolute_path))

        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.assertEqual(return_code, 0)

    def test_14_MpiCommandTemplate(self):
        """!
        Test if ns3 is inserting additional arguments by MPICH and OpenMPI to run on the CI
        @return None
        """
        # Skip test if mpi is not installed
        if shutil.which("mpiexec") is None:
            self.skipTest("Mpi is not available")

        # Ensure sample simulator was built
        return_code, stdout, stderr = run_ns3("build sample-simulator")
        self.assertEqual(return_code, 0)

        # Get executable path
        sample_simulator_path = list(filter(lambda x: "sample-simulator" in x, self.ns3_executables))[0]

        mpi_command = "--dry-run run sample-simulator --command-template=\"mpiexec -np 2 %s\""
        non_mpi_command = "--dry-run run sample-simulator --command-template=\"echo %s\""

        # Get the commands to run sample-simulator in two processes with mpi
        return_code, stdout, stderr = run_ns3(mpi_command)
        self.assertEqual(return_code, 0)
        self.assertIn("mpiexec -np 2 %s" % sample_simulator_path, stdout)

        # Get the commands to run sample-simulator in two processes with mpi, now with the environment variable
        return_code, stdout, stderr = run_ns3(mpi_command, env={"MPI_CI": "1"})
        self.assertEqual(return_code, 0)
        if shutil.which("ompi_info"):
            self.assertIn("mpiexec --allow-run-as-root --oversubscribe -np 2 %s" % sample_simulator_path, stdout)
        else:
            self.assertIn("mpiexec --allow-run-as-root -np 2 %s" % sample_simulator_path, stdout)

        # Now we repeat for the non-mpi command
        return_code, stdout, stderr = run_ns3(non_mpi_command)
        self.assertEqual(return_code, 0)
        self.assertIn("echo %s" % sample_simulator_path, stdout)

        # Again the non-mpi command, with the MPI_CI environment variable set
        return_code, stdout, stderr = run_ns3(non_mpi_command, env={"MPI_CI": "1"})
        self.assertEqual(return_code, 0)
        self.assertIn("echo %s" % sample_simulator_path, stdout)

    def test_15_InvalidLibrariesToLink(self):
        """!
        Test if CMake and ns3 fail in the expected ways when:
        - examples from modules or general examples fail if they depend on a
        library with a name shorter than 4 characters or are disabled when
        a library is nonexistent
        - a module library passes the configuration but fails to build due to
        a missing library
        @return None
        """
        os.makedirs("contrib/borked", exist_ok=True)
        os.makedirs("contrib/borked/examples", exist_ok=True)

        # Test if configuration succeeds and building the module library fails
        with open("contrib/borked/examples/CMakeLists.txt", "w") as f:
            f.write("")
        for invalid_or_nonexistent_library in ["", "gsd", "lib", "libfi", "calibre"]:
            with open("contrib/borked/CMakeLists.txt", "w") as f:
                f.write("""
                        build_lib(
                            LIBNAME borked
                            SOURCE_FILES ${PROJECT_SOURCE_DIR}/build-support/empty.cc
                            LIBRARIES_TO_LINK ${libcore} %s
                        )
                        """ % invalid_or_nonexistent_library)

            return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-examples")
            if invalid_or_nonexistent_library in ["", "gsd", "libfi", "calibre"]:
                self.assertEqual(return_code, 0)
            elif invalid_or_nonexistent_library in ["lib"]:
                self.assertEqual(return_code, 1)
                self.assertIn("Invalid library name: %s" % invalid_or_nonexistent_library, stderr)
            else:
                pass

            return_code, stdout, stderr = run_ns3("build borked")
            if invalid_or_nonexistent_library in [""]:
                self.assertEqual(return_code, 0)
            elif invalid_or_nonexistent_library in ["lib"]:
                self.assertEqual(return_code, 2)  # should fail due to invalid library name
                self.assertIn("Invalid library name: %s" % invalid_or_nonexistent_library, stderr)
            elif invalid_or_nonexistent_library in ["gsd", "libfi", "calibre"]:
                self.assertEqual(return_code, 2)  # should fail due to missing library
                self.assertIn("cannot find -l%s" % invalid_or_nonexistent_library, stderr)
            else:
                pass

        # Now test if the example can be built with:
        # - no additional library (should work)
        # - invalid library names (should fail to configure)
        # - valid library names but nonexistent libraries (should not create a target)
        with open("contrib/borked/CMakeLists.txt", "w") as f:
            f.write("""
                    build_lib(
                        LIBNAME borked
                        SOURCE_FILES ${PROJECT_SOURCE_DIR}/build-support/empty.cc
                        LIBRARIES_TO_LINK ${libcore}
                    )
                    """)
        for invalid_or_nonexistent_library in ["", "gsd", "lib", "libfi", "calibre"]:
            with open("contrib/borked/examples/CMakeLists.txt", "w") as f:
                f.write("""
                        build_lib_example(
                            NAME borked-example
                            SOURCE_FILES ${PROJECT_SOURCE_DIR}/build-support/empty-main.cc
                            LIBRARIES_TO_LINK ${libborked} %s
                        )
                        """ % invalid_or_nonexistent_library)

            return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
            if invalid_or_nonexistent_library in ["", "gsd", "libfi", "calibre"]:
                self.assertEqual(return_code, 0)  # should be able to configure
            elif invalid_or_nonexistent_library in ["lib"]:
                self.assertEqual(return_code, 1)  # should fail to even configure
                self.assertIn("Invalid library name: %s" % invalid_or_nonexistent_library, stderr)
            else:
                pass

            return_code, stdout, stderr = run_ns3("build borked-example")
            if invalid_or_nonexistent_library in [""]:
                self.assertEqual(return_code, 0)  # should be able to build
            elif invalid_or_nonexistent_library in ["libf"]:
                self.assertEqual(return_code, 2)  # should fail due to missing configuration
                self.assertIn("Invalid library name: %s" % invalid_or_nonexistent_library, stderr)
            elif invalid_or_nonexistent_library in ["gsd", "libfi", "calibre"]:
                self.assertEqual(return_code, 1)  # should fail to find target
                self.assertIn("Target to build does not exist: borked-example", stdout)
            else:
                pass

        shutil.rmtree("contrib/borked", ignore_errors=True)

    def test_16_LibrariesContainingLib(self):
        """!
        Test if CMake can properly handle modules containing "lib",
        which is used internally as a prefix for module libraries
        @return None
        """

        os.makedirs("contrib/calibre", exist_ok=True)
        os.makedirs("contrib/calibre/examples", exist_ok=True)

        # Now test if we can have a library with "lib" in it
        with open("contrib/calibre/examples/CMakeLists.txt", "w") as f:
            f.write("")
        with open("contrib/calibre/CMakeLists.txt", "w") as f:
            f.write("""
                build_lib(
                    LIBNAME calibre
                    SOURCE_FILES ${PROJECT_SOURCE_DIR}/build-support/empty.cc
                    LIBRARIES_TO_LINK ${libcore}
                )
                """)

        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")

        # This only checks if configuration passes
        self.assertEqual(return_code, 0)

        # This checks if the contrib modules were printed correctly
        self.assertIn("calibre", stdout)

        # This checks not only if "lib" from "calibre" was incorrectly removed,
        # but also if the pkgconfig file was generated with the correct name
        self.assertNotIn("care", stdout)
        self.assertTrue(os.path.exists(os.path.join(ns3_path, "cmake-cache", "pkgconfig", "ns3-calibre.pc")))

        # Check if we can build this library
        return_code, stdout, stderr = run_ns3("build calibre")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_target_command(target="libcalibre"), stdout)

        shutil.rmtree("contrib/calibre", ignore_errors=True)

    def test_17_CMakePerformanceTracing(self):
        """!
        Test if CMake performance tracing works and produces the
        cmake_performance_trace.log file
        @return None
        """
        return_code, stdout, stderr = run_ns3("configure --trace-performance")
        self.assertEqual(return_code, 0)
        self.assertIn("--profiling-format=google-trace --profiling-output=../cmake_performance_trace.log", stdout)
        self.assertTrue(os.path.exists(os.path.join(ns3_path, "cmake_performance_trace.log")))

    def test_18_CheckBuildVersionAndVersionCache(self):
        """!
        Check if ENABLE_BUILD_VERSION and version.cache are working
        as expected
        @return None
        """

        try:
            from python_on_whales import docker
            from python_on_whales.exceptions import DockerException
        except ModuleNotFoundError:
            self.skipTest("python-on-whales was not found")

        # Import rootless docker settings from .bashrc
        with open(os.path.expanduser("~/.bashrc"), "r") as f:
            docker_settings = re.findall("(DOCKER_.*=.*)", f.read())
            for setting in docker_settings:
                key, value = setting.split("=")
                os.environ[key] = value
            del docker_settings, setting, key, value

        # Create Docker client instance and start it
        with docker.run("ubuntu:22.04",
                        interactive=True, detach=True,
                        tty=False,
                        volumes=[(ns3_path, "/ns-3-dev")]
                        ) as container:
            # Redefine the execute command of the container
            def split_exec(self, cmd):
                return self._execute(cmd.split(), workdir="/ns-3-dev")

            container._execute = container.execute
            container.execute = partial(split_exec, container)

            # Install basic packages
            container.execute("apt-get update")
            container.execute("apt-get install -y python3 ninja-build cmake g++")

            # Clean ns-3 artifacts
            container.execute("./ns3 clean")

            # Set path to version.cache file
            version_cache_file = os.path.join(ns3_path, "src/core/model/version.cache")

            # First case: try without a version cache or Git
            if os.path.exists(version_cache_file):
                os.remove(version_cache_file)

            # We need to catch the exception since the command will fail
            try:
                container.execute("./ns3 configure -G Ninja --enable-build-version")
            except DockerException:
                pass
            self.assertFalse(os.path.exists(os.path.join(ns3_path, "cmake-cache", "build.ninja")))

            # Second case: try with a version cache file but without Git (it should succeed)
            version_cache_contents = ("CLOSEST_TAG = '\"ns-3.0.0\"'\n"
                                      "VERSION_COMMIT_HASH = '\"0000000000\"'\n"
                                      "VERSION_DIRTY_FLAG = '0'\n"
                                      "VERSION_MAJOR = '3'\n"
                                      "VERSION_MINOR = '0'\n"
                                      "VERSION_PATCH = '0'\n"
                                      "VERSION_RELEASE_CANDIDATE = '\"\"'\n"
                                      "VERSION_TAG = '\"ns-3.0.0\"'\n"
                                      "VERSION_TAG_DISTANCE = '0'\n"
                                      "VERSION_BUILD_PROFILE = 'debug'\n"
                                      )
            with open(version_cache_file, "w") as version:
                version.write(version_cache_contents)

            # Configuration should now succeed
            container.execute("./ns3 clean")
            container.execute("./ns3 configure -G Ninja --enable-build-version")
            container.execute("./ns3 build core")
            self.assertTrue(os.path.exists(os.path.join(ns3_path, "cmake-cache", "build.ninja")))

            # And contents of version cache should be unchanged
            with open(version_cache_file, "r") as version:
                self.assertEqual(version.read(), version_cache_contents)

            # Third case: we rename the .git directory temporarily and reconfigure
            # to check if it gets configured successfully when Git is found but
            # there is not .git history
            os.rename(os.path.join(ns3_path, ".git"), os.path.join(ns3_path, "temp_git"))
            try:
                container.execute("apt-get install -y git")
                container.execute("./ns3 clean")
                container.execute("./ns3 configure -G Ninja --enable-build-version")
                container.execute("./ns3 build core")
            except Exception:
                pass
            os.rename(os.path.join(ns3_path, "temp_git"), os.path.join(ns3_path, ".git"))
            self.assertTrue(os.path.exists(os.path.join(ns3_path, "cmake-cache", "build.ninja")))

            # Fourth case: test with Git and git history. Now the version.cache should be replaced.
            container.execute("./ns3 clean")
            container.execute("./ns3 configure -G Ninja --enable-build-version")
            container.execute("./ns3 build core")
            self.assertTrue(os.path.exists(os.path.join(ns3_path, "cmake-cache", "build.ninja")))
            with open(version_cache_file, "r") as version:
                self.assertNotEqual(version.read(), version_cache_contents)


class NS3BuildBaseTestCase(NS3BaseTestCase):
    """!
    Tests ns3 regarding building the project
    """

    ## when cleaned_once is False, clean up build artifacts and reconfigure # noqa
    cleaned_once = False

    def setUp(self):
        """!
        Reuse cleaning/release configuration from NS3BaseTestCase if flag is cleaned
        @return None
        """
        if not NS3BuildBaseTestCase.cleaned_once:
            NS3BuildBaseTestCase.cleaned_once = True
            NS3BaseTestCase.cleaned_once = False
        super().setUp()

        self.ns3_libraries = get_libraries_list()

    def test_01_BuildExistingTargets(self):
        """!
        Try building the core library
        @return None
        """
        return_code, stdout, stderr = run_ns3("build core")
        self.assertEqual(return_code, 0)
        self.assertIn("Built target libcore", stdout)

    def test_02_BuildNonExistingTargets(self):
        """!
        Try building core-test library without tests enabled
        @return None
        """
        # tests are not enabled, so the target isn't available
        return_code, stdout, stderr = run_ns3("build core-test")
        self.assertEqual(return_code, 1)
        self.assertIn("Target to build does not exist: core-test", stdout)

    def test_03_BuildProject(self):
        """!
        Try building the project:
        @return None
        """
        return_code, stdout, stderr = run_ns3("build")
        self.assertEqual(return_code, 0)
        self.assertIn("Built target", stdout)
        for program in get_programs_list():
            self.assertTrue(os.path.exists(program))
        self.assertIn(cmake_build_project_command, stdout)

    def test_04_BuildProjectNoTaskLines(self):
        """!
        Try hiding task lines
        @return None
        """
        return_code, stdout, stderr = run_ns3("--quiet build")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_project_command, stdout)

    def test_05_BreakBuild(self):
        """!
        Try removing an essential file to break the build
        @return None
        """
        # change an essential file to break the build.
        attribute_cc_path = os.sep.join([ns3_path, "src", "core", "model", "attribute.cc"])
        attribute_cc_bak_path = attribute_cc_path + ".bak"
        shutil.move(attribute_cc_path, attribute_cc_bak_path)

        # build should break.
        return_code, stdout, stderr = run_ns3("build")
        self.assertNotEqual(return_code, 0)

        # move file back.
        shutil.move(attribute_cc_bak_path, attribute_cc_path)

        # Build should work again.
        return_code, stdout, stderr = run_ns3("build")
        self.assertEqual(return_code, 0)

        # Reset flag to let it clean the build
        NS3BuildBaseTestCase.cleaned_once = False

    def test_06_TestVersionFile(self):
        """!
        Test if changing the version file affects the library names
        @return None
        """
        run_ns3("build")
        self.ns3_libraries = get_libraries_list()

        version_file = os.sep.join([ns3_path, "VERSION"])
        with open(version_file, "w") as f:
            f.write("3-00\n")

        # Reconfigure.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\"")
        self.config_ok(return_code, stdout)

        # Build.
        return_code, stdout, stderr = run_ns3("build")
        self.assertEqual(return_code, 0)
        self.assertIn("Built target", stdout)

        # Programs with new versions.
        new_programs = get_programs_list()

        # Check if they exist.
        for program in new_programs:
            self.assertTrue(os.path.exists(program))

        # Check if we still have the same number of binaries.
        self.assertEqual(len(new_programs), len(self.ns3_executables))

        # Check if versions changed from 3-dev to 3-00.
        libraries = get_libraries_list()
        new_libraries = list(set(libraries).difference(set(self.ns3_libraries)))
        self.assertEqual(len(new_libraries), len(self.ns3_libraries))
        for library in new_libraries:
            self.assertNotIn("libns3-dev", library)
            self.assertIn("libns3-00", library)
            self.assertTrue(os.path.exists(library))

        # Restore version file.
        with open(version_file, "w") as f:
            f.write("3-dev\n")

        # Reset flag to let it clean the build.
        NS3BuildBaseTestCase.cleaned_once = False

    def test_07_OutputDirectory(self):
        """!
        Try setting a different output directory and if everything is
        in the right place and still working correctly
        @return None
        """
        # Re-build to return to the original state.
        run_ns3("build")

        ## ns3_libraries holds a list of built module libraries # noqa
        self.ns3_libraries = get_libraries_list()

        ## ns3_executables holds a list of executables in .lock-ns3 # noqa
        self.ns3_executables = get_programs_list()

        # Delete built programs and libraries to check if they were restored later.
        for program in self.ns3_executables:
            os.remove(program)
        for library in self.ns3_libraries:
            os.remove(library)

        # Reconfigure setting the output folder to ns-3-dev/build/release (both as an absolute path or relative).
        absolute_path = os.sep.join([ns3_path, "build", "release"])
        relative_path = os.sep.join(["build", "release"])
        for different_out_dir in [absolute_path, relative_path]:
            return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --out=%s" % different_out_dir)
            self.config_ok(return_code, stdout)
            self.assertIn("Build directory               : %s" % absolute_path, stdout)

            # Build
            run_ns3("build")

            # Check if we have the same number of binaries and that they were built correctly.
            new_programs = get_programs_list()
            self.assertEqual(len(new_programs), len(self.ns3_executables))
            for program in new_programs:
                self.assertTrue(os.path.exists(program))

            # Check if we have the same number of libraries and that they were built correctly.
            libraries = get_libraries_list(os.sep.join([absolute_path, "lib"]))
            new_libraries = list(set(libraries).difference(set(self.ns3_libraries)))
            self.assertEqual(len(new_libraries), len(self.ns3_libraries))
            for library in new_libraries:
                self.assertTrue(os.path.exists(library))

            # Remove files in the different output dir.
            shutil.rmtree(absolute_path)

        # Restore original output directory.
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --out=''")
        self.config_ok(return_code, stdout)
        self.assertIn("Build directory               : %s" % usual_outdir, stdout)

        # Try re-building.
        run_ns3("build")

        # Check if we have the same binaries we had at the beginning.
        new_programs = get_programs_list()
        self.assertEqual(len(new_programs), len(self.ns3_executables))
        for program in new_programs:
            self.assertTrue(os.path.exists(program))

        # Check if we have the same libraries we had at the beginning.
        libraries = get_libraries_list()
        self.assertEqual(len(libraries), len(self.ns3_libraries))
        for library in libraries:
            self.assertTrue(os.path.exists(library))

    def test_08_InstallationAndUninstallation(self):
        """!
        Tries setting a ns3 version, then installing it.
        After that, tries searching for ns-3 with CMake's find_package(ns3).
        Finally, tries using core library in a 3rd-party project
        @return None
        """
        # Remove existing libraries from the previous step.
        libraries = get_libraries_list()
        for library in libraries:
            os.remove(library)

        # 3-dev version format is not supported by CMake, so we use 3.01.
        version_file = os.sep.join([ns3_path, "VERSION"])
        with open(version_file, "w") as f:
            f.write("3-01\n")

        # Reconfigure setting the installation folder to ns-3-dev/build/install.
        install_prefix = os.sep.join([ns3_path, "build", "install"])
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --prefix=%s" % install_prefix)
        self.config_ok(return_code, stdout)

        # Build.
        run_ns3("build")
        libraries = get_libraries_list()
        headers = get_headers_list()

        # Install.
        run_ns3("install")

        # Find out if libraries were installed to lib or lib64 (Fedora thing).
        lib64 = os.path.exists(os.sep.join([install_prefix, "lib64"]))
        installed_libdir = os.sep.join([install_prefix, ("lib64" if lib64 else "lib")])

        # Make sure all libraries were installed.
        installed_libraries = get_libraries_list(installed_libdir)
        installed_libraries_list = ";".join(installed_libraries)
        for library in libraries:
            library_name = os.path.basename(library)
            self.assertIn(library_name, installed_libraries_list)

        # Make sure all headers were installed.
        installed_headers = get_headers_list(install_prefix)
        missing_headers = list(set([os.path.basename(x) for x in headers])
                               - (set([os.path.basename(x) for x in installed_headers]))
                               )
        self.assertEqual(len(missing_headers), 0)

        # Now create a test CMake project and try to find_package ns-3.
        test_main_file = os.sep.join([install_prefix, "main.cpp"])
        with open(test_main_file, "w") as f:
            f.write("""
            #include <ns3/core-module.h>
            using namespace ns3;
            int main ()
            {
                Simulator::Stop (Seconds (1.0));
                Simulator::Run ();
                Simulator::Destroy ();
                return 0;
            }
            """)

        # We try to use this library without specifying a version,
        # specifying ns3-01 (text version with 'dev' is not supported)
        # and specifying ns3-00 (a wrong version)
        for version in ["", "3.01", "3.00"]:
            ns3_import_methods = []

            # Import ns-3 libraries with as a CMake package
            cmake_find_package_import = """
                                  list(APPEND CMAKE_PREFIX_PATH ./{lib}/cmake/ns3)
                                  find_package(ns3 {version} COMPONENTS libcore)
                                  target_link_libraries(test PRIVATE ns3::libcore)
                                  """.format(lib=("lib64" if lib64 else "lib"), version=version)
            ns3_import_methods.append(cmake_find_package_import)

            # Import ns-3 as pkg-config libraries
            pkgconfig_import = """
                               list(APPEND CMAKE_PREFIX_PATH ./)
                               include(FindPkgConfig)
                               pkg_check_modules(ns3 REQUIRED IMPORTED_TARGET ns3-core{version})
                               target_link_libraries(test PUBLIC PkgConfig::ns3)
                               """.format(lib=("lib64" if lib64 else "lib"),
                                          version="=" + version if version else ""
                                          )
            if shutil.which("pkg-config"):
                ns3_import_methods.append(pkgconfig_import)

            # Test the multiple ways of importing ns-3 libraries
            for import_method in ns3_import_methods:
                test_cmake_project = """
                                     cmake_minimum_required(VERSION 3.10..3.10)
                                     project(ns3_consumer CXX)
                                     set(CMAKE_CXX_STANDARD 17)
                                     set(CMAKE_CXX_STANDARD_REQUIRED ON)
                                     add_executable(test main.cpp)
                                     """ + import_method

                test_cmake_project_file = os.sep.join([install_prefix, "CMakeLists.txt"])
                with open(test_cmake_project_file, "w") as f:
                    f.write(test_cmake_project)

                # Configure the test project
                cmake = shutil.which("cmake")
                return_code, stdout, stderr = run_program(cmake,
                                                          "-DCMAKE_BUILD_TYPE=debug .",
                                                          cwd=install_prefix)
                if version == "3.00":
                    self.assertEqual(return_code, 1)
                    if import_method == cmake_find_package_import:
                        self.assertIn('Could not find a configuration file for package "ns3" that is compatible',
                                      stderr.replace("\n", ""))
                    elif import_method == pkgconfig_import:
                        self.assertIn('A required package was not found',
                                      stderr.replace("\n", ""))
                    else:
                        raise Exception("Unknown import type")
                else:
                    self.assertEqual(return_code, 0)
                    self.assertIn("Build files", stdout)

                # Build the test project making use of import ns-3
                return_code, stdout, stderr = run_program("cmake", "--build .", cwd=install_prefix)

                if version == "3.00":
                    self.assertEqual(return_code, 2)
                    self.assertGreater(len(stderr), 0)
                else:
                    self.assertEqual(return_code, 0)
                    self.assertIn("Built target", stdout)

                    # Try running the test program that imports ns-3
                    return_code, stdout, stderr = run_program("./test", "", cwd=install_prefix)
                    self.assertEqual(return_code, 0)

        # Uninstall
        return_code, stdout, stderr = run_ns3("uninstall")
        self.assertIn("Built target uninstall", stdout)

        # Restore 3-dev version file
        with open(version_file, "w") as f:
            f.write("3-dev\n")

        # Reset flag to let it clean the build
        NS3BuildBaseTestCase.cleaned_once = False

    def test_09_Scratches(self):
        """!
        Tries to build scratch-simulator and subdir/scratch-simulator-subdir
        @return None
        """
        # Build.
        targets = {"scratch/scratch-simulator": "scratch-simulator",
                   "scratch/scratch-simulator.cc": "scratch-simulator",
                   "scratch-simulator": "scratch-simulator",
                   "scratch/subdir/scratch-simulator-subdir": "subdir_scratch-simulator-subdir",
                   "subdir/scratch-simulator-subdir": "subdir_scratch-simulator-subdir",
                   "scratch-simulator-subdir": "subdir_scratch-simulator-subdir",
                   }
        for (target_to_run, target_cmake) in targets.items():
            # Test if build is working.
            build_line = "target scratch_%s" % target_cmake
            return_code, stdout, stderr = run_ns3("build %s" % target_to_run)
            self.assertEqual(return_code, 0)
            self.assertIn(build_line, stdout)

            # Test if run is working
            return_code, stdout, stderr = run_ns3("run %s --verbose" % target_to_run)
            self.assertEqual(return_code, 0)
            self.assertIn(build_line, stdout)
            stdout = stdout.replace("scratch_%s" % target_cmake, "")  # remove build lines
            self.assertIn(target_to_run.split("/")[-1].replace(".cc", ""), stdout)

        NS3BuildBaseTestCase.cleaned_once = False

    def test_10_AmbiguityCheck(self):
        """!
        Test if ns3 can alert correctly in case a shortcut collision happens
        @return None
        """

        # First enable examples
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-examples")
        self.assertEqual(return_code, 0)

        # Copy second.cc from the tutorial examples to the scratch folder
        shutil.copy("./examples/tutorial/second.cc", "./scratch/second.cc")

        # Reconfigure to re-scan the scratches
        return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-examples")
        self.assertEqual(return_code, 0)

        # Try to run second and collide
        return_code, stdout, stderr = run_ns3("build second")
        self.assertEqual(return_code, 1)
        self.assertIn(
            'Build target "second" is ambiguous. Try one of these: "scratch/second", "examples/tutorial/second"',
            stdout
        )

        # Try to run scratch/second and succeed
        return_code, stdout, stderr = run_ns3("build scratch/second")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_target_command(target="scratch_second"), stdout)

        # Try to run scratch/second and succeed
        return_code, stdout, stderr = run_ns3("build tutorial/second")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_target_command(target="second"), stdout)

        # Try to run second and collide
        return_code, stdout, stderr = run_ns3("run second")
        self.assertEqual(return_code, 1)
        self.assertIn(
            'Run target "second" is ambiguous. Try one of these: "scratch/second", "examples/tutorial/second"',
            stdout
        )

        # Try to run scratch/second and succeed
        return_code, stdout, stderr = run_ns3("run scratch/second")
        self.assertEqual(return_code, 0)

        # Try to run scratch/second and succeed
        return_code, stdout, stderr = run_ns3("run tutorial/second")
        self.assertEqual(return_code, 0)

        # Remove second
        os.remove("./scratch/second.cc")

        NS3BuildBaseTestCase.cleaned_once = False

    def test_11_StaticBuilds(self):
        """!
        Test if we can build a static ns-3 library and link it to static programs
        @return None
        """
        # First enable examples and static build
        return_code, stdout, stderr = run_ns3(
            "configure -G \"Unix Makefiles\" --enable-examples --disable-gtk --enable-static")

        # If configuration passes, we are half way done
        self.assertEqual(return_code, 0)

        # Then try to build one example
        return_code, stdout, stderr = run_ns3('build sample-simulator')
        self.assertEqual(return_code, 0)
        self.assertIn("Built target", stdout)

        # Maybe check the built binary for shared library references? Using objdump, otool, etc
        NS3BuildBaseTestCase.cleaned_once = False

    def test_12_CppyyBindings(self):
        """!
        Test if we can use python bindings
        @return None
        """
        try:
            import cppyy
        except ModuleNotFound:
            self.skipTest("Cppyy was not found")

        # First enable examples and static build
        return_code, stdout, stderr = run_ns3(
            "configure -G \"Unix Makefiles\" --enable-python-bindings")

        # If configuration passes, we are half way done
        self.assertEqual(return_code, 0)

        # Then build and run tests
        return_code, stdout, stderr = run_program("test.py", "", python=True)
        self.assertEqual(return_code, 0)

        # Then try to run a specific test
        return_code, stdout, stderr = run_program("test.py", "-p mixed-wired-wireless", python=True)
        self.assertEqual(return_code, 0)

        # Then try to run a specific test with the full relative path
        return_code, stdout, stderr = run_program("test.py", "-p ./examples/wireless/mixed-wired-wireless", python=True)
        self.assertEqual(return_code, 0)


class NS3ExpectedUseTestCase(NS3BaseTestCase):
    """!
    Tests ns3 usage in more realistic scenarios
    """

    ## when cleaned_once is False, clean up build artifacts and reconfigure # noqa
    cleaned_once = False

    def setUp(self):
        """!
        Reuse cleaning/release configuration from NS3BaseTestCase if flag is cleaned
        Here examples, tests and documentation are also enabled.
        @return None
        """
        if not NS3ExpectedUseTestCase.cleaned_once:
            NS3ExpectedUseTestCase.cleaned_once = True
            NS3BaseTestCase.cleaned_once = False
            super().setUp()

            # On top of the release build configured by NS3ConfigureTestCase, also enable examples, tests and docs.
            return_code, stdout, stderr = run_ns3("configure -G \"Unix Makefiles\" --enable-examples --enable-tests")
            self.config_ok(return_code, stdout)

        # Check if .lock-ns3 exists, then read to get list of executables.
        self.assertTrue(os.path.exists(ns3_lock_filename))

        ## ns3_executables holds a list of executables in .lock-ns3 # noqa
        self.ns3_executables = get_programs_list()

        # Check if .lock-ns3 exists than read to get the list of enabled modules.
        self.assertTrue(os.path.exists(ns3_lock_filename))

        ## ns3_modules holds a list to the modules enabled stored in .lock-ns3 # noqa
        self.ns3_modules = get_enabled_modules()

    def test_01_BuildProject(self):
        """!
        Try to build the project
        @return None
        """
        return_code, stdout, stderr = run_ns3("build")
        self.assertEqual(return_code, 0)
        self.assertIn("Built target", stdout)
        for program in get_programs_list():
            self.assertTrue(os.path.exists(program))
        libraries = get_libraries_list()
        for module in get_enabled_modules():
            self.assertIn(module.replace("ns3-", ""), ";".join(libraries))
        self.assertIn(cmake_build_project_command, stdout)

    def test_02_BuildAndRunExistingExecutableTarget(self):
        """!
        Try to build and run test-runner
        @return None
        """
        return_code, stdout, stderr = run_ns3('run "test-runner --list" --verbose')
        self.assertEqual(return_code, 0)
        self.assertIn("Built target test-runner", stdout)
        self.assertIn(cmake_build_target_command(target="test-runner"), stdout)

    def test_03_BuildAndRunExistingLibraryTarget(self):
        """!
        Try to build and run a library
        @return None
        """
        return_code, stdout, stderr = run_ns3("run core")  # this should not work
        self.assertEqual(return_code, 1)
        self.assertIn("Couldn't find the specified program: core", stderr)

    def test_04_BuildAndRunNonExistingTarget(self):
        """!
        Try to build and run an unknown target
        @return None
        """
        return_code, stdout, stderr = run_ns3("run nonsense")  # this should not work
        self.assertEqual(return_code, 1)
        self.assertIn("Couldn't find the specified program: nonsense", stderr)

    def test_05_RunNoBuildExistingExecutableTarget(self):
        """!
        Try to run test-runner without building
        @return None
        """
        return_code, stdout, stderr = run_ns3('run "test-runner --list" --no-build --verbose')
        self.assertEqual(return_code, 0)
        self.assertNotIn("Built target test-runner", stdout)
        self.assertNotIn(cmake_build_target_command(target="test-runner"), stdout)

    def test_06_RunNoBuildExistingLibraryTarget(self):
        """!
        Test ns3 fails to run a library
        @return None
        """
        return_code, stdout, stderr = run_ns3("run core --no-build")  # this should not work
        self.assertEqual(return_code, 1)
        self.assertIn("Couldn't find the specified program: core", stderr)

    def test_07_RunNoBuildNonExistingExecutableTarget(self):
        """!
        Test ns3 fails to run an unknown program
        @return None
        """
        return_code, stdout, stderr = run_ns3("run nonsense --no-build")  # this should not work
        self.assertEqual(return_code, 1)
        self.assertIn("Couldn't find the specified program: nonsense", stderr)

    def test_08_RunNoBuildGdb(self):
        """!
        Test if scratch simulator is executed through gdb and lldb
        @return None
        """
        if shutil.which("gdb") is None:
            self.skipTest("Missing gdb")

        return_code, stdout, stderr = run_ns3("run scratch-simulator --gdb --verbose --no-build")
        self.assertEqual(return_code, 0)
        self.assertIn("scratch-simulator", stdout)
        self.assertIn("No debugging symbols found", stdout)

    def test_09_RunNoBuildValgrind(self):
        """!
        Test if scratch simulator is executed through valgrind
        @return None
        """
        if shutil.which("valgrind") is None:
            self.skipTest("Missing valgrind")

        return_code, stdout, stderr = run_ns3("run scratch-simulator --valgrind --verbose --no-build")
        self.assertEqual(return_code, 0)
        self.assertIn("scratch-simulator", stderr)
        self.assertIn("Memcheck", stderr)

    def test_10_DoxygenWithBuild(self):
        """!
        Test the doxygen target that does trigger a full build
        @return None
        """
        if shutil.which("doxygen") is None:
            self.skipTest("Missing doxygen")

        doc_folder = os.path.abspath(os.sep.join([".", "doc"]))

        doxygen_files = ["introspected-command-line.h", "introspected-doxygen.h"]
        for filename in doxygen_files:
            file_path = os.sep.join([doc_folder, filename])
            if os.path.exists(file_path):
                os.remove(file_path)

        # Rebuilding dot images is super slow, so not removing doxygen products
        # doxygen_build_folder = os.sep.join([doc_folder, "html"])
        # if os.path.exists(doxygen_build_folder):
        #     shutil.rmtree(doxygen_build_folder)

        return_code, stdout, stderr = run_ns3("docs doxygen")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_target_command(target="doxygen"), stdout)
        self.assertIn("Built target doxygen", stdout)

    def test_11_DoxygenWithoutBuild(self):
        """!
        Test the doxygen target that doesn't trigger a full build
        @return None
        """
        if shutil.which("doxygen") is None:
            self.skipTest("Missing doxygen")

        # Rebuilding dot images is super slow, so not removing doxygen products
        # doc_folder = os.path.abspath(os.sep.join([".", "doc"]))
        # doxygen_build_folder = os.sep.join([doc_folder, "html"])
        # if os.path.exists(doxygen_build_folder):
        #     shutil.rmtree(doxygen_build_folder)

        return_code, stdout, stderr = run_ns3("docs doxygen-no-build")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_target_command(target="doxygen-no-build"), stdout)
        self.assertIn("Built target doxygen-no-build", stdout)

    def test_12_SphinxDocumentation(self):
        """!
        Test every individual target for Sphinx-based documentation
        @return None
        """
        if shutil.which("sphinx-build") is None:
            self.skipTest("Missing sphinx")

        doc_folder = os.path.abspath(os.sep.join([".", "doc"]))

        # For each sphinx doc target.
        for target in ["contributing", "manual", "models", "tutorial"]:
            # First we need to clean old docs, or it will not make any sense.
            doc_build_folder = os.sep.join([doc_folder, target, "build"])
            doc_temp_folder = os.sep.join([doc_folder, target, "source-temp"])
            if os.path.exists(doc_build_folder):
                shutil.rmtree(doc_build_folder)
            if os.path.exists(doc_temp_folder):
                shutil.rmtree(doc_temp_folder)

            # Build
            return_code, stdout, stderr = run_ns3("docs %s" % target)
            self.assertEqual(return_code, 0, target)
            self.assertIn(cmake_build_target_command(target="sphinx_%s" % target), stdout)
            self.assertIn("Built target sphinx_%s" % target, stdout)

            # Check if the docs output folder exists
            doc_build_folder = os.sep.join([doc_folder, target, "build"])
            self.assertTrue(os.path.exists(doc_build_folder))

            # Check if the all the different types are in place (latex, split HTML and single page HTML)
            for build_type in ["latex", "html", "singlehtml"]:
                self.assertTrue(os.path.exists(os.sep.join([doc_build_folder, build_type])))

    def test_13_Documentation(self):
        """!
        Test the documentation target that builds
        both doxygen and sphinx based documentation
        @return None
        """
        if shutil.which("doxygen") is None:
            self.skipTest("Missing doxygen")
        if shutil.which("sphinx-build") is None:
            self.skipTest("Missing sphinx")

        doc_folder = os.path.abspath(os.sep.join([".", "doc"]))

        # First we need to clean old docs, or it will not make any sense.

        # Rebuilding dot images is super slow, so not removing doxygen products
        # doxygen_build_folder = os.sep.join([doc_folder, "html"])
        # if os.path.exists(doxygen_build_folder):
        #     shutil.rmtree(doxygen_build_folder)

        for target in ["manual", "models", "tutorial"]:
            doc_build_folder = os.sep.join([doc_folder, target, "build"])
            if os.path.exists(doc_build_folder):
                shutil.rmtree(doc_build_folder)

        return_code, stdout, stderr = run_ns3("docs all")
        self.assertEqual(return_code, 0)
        self.assertIn(cmake_build_target_command(target="sphinx"), stdout)
        self.assertIn("Built target sphinx", stdout)
        self.assertIn(cmake_build_target_command(target="doxygen"), stdout)
        self.assertIn("Built target doxygen", stdout)

    def test_14_EnableSudo(self):
        """!
        Try to set ownership of scratch-simulator from current user to root,
        and change execution permissions
        @return None
        """

        # Test will be skipped if not defined
        sudo_password = os.getenv("SUDO_PASSWORD", None)

        # Skip test if variable containing sudo password is the default value
        if sudo_password is None:
            self.skipTest("SUDO_PASSWORD environment variable was not specified")

        enable_sudo = read_lock_entry("ENABLE_SUDO")
        self.assertFalse(enable_sudo is True)

        # First we run to ensure the program was built
        return_code, stdout, stderr = run_ns3('run scratch-simulator')
        self.assertEqual(return_code, 0)
        self.assertIn("Built target scratch_scratch-simulator", stdout)
        self.assertIn(cmake_build_target_command(target="scratch_scratch-simulator"), stdout)
        scratch_simulator_path = list(filter(lambda x: x if "scratch-simulator" in x else None,
                                             self.ns3_executables
                                             )
                                      )[-1]
        prev_fstat = os.stat(scratch_simulator_path)  # we get the permissions before enabling sudo

        # Now try setting the sudo bits from the run subparser
        return_code, stdout, stderr = run_ns3('run scratch-simulator --enable-sudo',
                                              env={"SUDO_PASSWORD": sudo_password})
        self.assertEqual(return_code, 0)
        self.assertIn("Built target scratch_scratch-simulator", stdout)
        self.assertIn(cmake_build_target_command(target="scratch_scratch-simulator"), stdout)
        fstat = os.stat(scratch_simulator_path)

        import stat
        # If we are on Windows, these permissions mean absolutely nothing,
        # and on Fuse builds they might not make any sense, so we need to skip before failing
        likely_fuse_mount = ((prev_fstat.st_mode & stat.S_ISUID) == (fstat.st_mode & stat.S_ISUID)) and \
                            prev_fstat.st_uid == 0  # noqa

        if sys.platform == "win32" or likely_fuse_mount:
            self.skipTest("Windows or likely a FUSE mount")

        # If this is a valid platform, we can continue
        self.assertEqual(fstat.st_uid, 0)  # check the file was correctly chown'ed by root
        self.assertEqual(fstat.st_mode & stat.S_ISUID, stat.S_ISUID)  # check if normal users can run as sudo

        # Now try setting the sudo bits as a post-build step (as set by configure subparser)
        return_code, stdout, stderr = run_ns3('configure --enable-sudo')
        self.assertEqual(return_code, 0)

        # Check if it was properly set in the lock file
        enable_sudo = read_lock_entry("ENABLE_SUDO")
        self.assertTrue(enable_sudo)

        # Remove old executables
        for executable in self.ns3_executables:
            if os.path.exists(executable):
                os.remove(executable)

        # Try to build and then set sudo bits as a post-build step
        return_code, stdout, stderr = run_ns3('build', env={"SUDO_PASSWORD": sudo_password})
        self.assertEqual(return_code, 0)

        # Check if commands are being printed for every target
        self.assertIn("chown root", stdout)
        self.assertIn("chmod u+s", stdout)
        for executable in self.ns3_executables:
            self.assertIn(os.path.basename(executable), stdout)

        # Check scratch simulator yet again
        fstat = os.stat(scratch_simulator_path)
        self.assertEqual(fstat.st_uid, 0)  # check the file was correctly chown'ed by root
        self.assertEqual(fstat.st_mode & stat.S_ISUID, stat.S_ISUID)  # check if normal users can run as sudo

    def test_15_CommandTemplate(self):
        """!
        Check if command template is working
        @return None
        """

        # Command templates that are empty or do not have a '%s' should fail
        return_code0, stdout0, stderr0 = run_ns3('run sample-simulator --command-template')
        self.assertEqual(return_code0, 2)
        self.assertIn("argument --command-template: expected one argument", stderr0)

        return_code1, stdout1, stderr1 = run_ns3('run sample-simulator --command-template=" "')
        return_code2, stdout2, stderr2 = run_ns3('run sample-simulator --command-template " "')
        return_code3, stdout3, stderr3 = run_ns3('run sample-simulator --command-template "echo "')
        self.assertEqual((return_code1, return_code2, return_code3), (1, 1, 1))
        self.assertIn("not all arguments converted during string formatting", stderr1)
        self.assertEqual(stderr1, stderr2)
        self.assertEqual(stderr2, stderr3)

        # Command templates with %s should at least continue and try to run the target
        return_code4, stdout4, _ = run_ns3('run sample-simulator --command-template "%s --PrintVersion" --verbose')
        return_code5, stdout5, _ = run_ns3('run sample-simulator --command-template="%s --PrintVersion" --verbose')
        self.assertEqual((return_code4, return_code5), (0, 0))
        self.assertIn("sample-simulator --PrintVersion", stdout4)
        self.assertIn("sample-simulator --PrintVersion", stdout5)

    def test_16_ForwardArgumentsToRunTargets(self):
        """!
        Check if all flavors of different argument passing to
        executable targets are working
        @return None
        """

        # Test if all argument passing flavors are working
        return_code0, stdout0, stderr0 = run_ns3('run "sample-simulator --help" --verbose')
        return_code1, stdout1, stderr1 = run_ns3('run sample-simulator --command-template="%s --help" --verbose')
        return_code2, stdout2, stderr2 = run_ns3('run sample-simulator --verbose -- --help')

        self.assertEqual((return_code0, return_code1, return_code2), (0, 0, 0))
        self.assertIn("sample-simulator --help", stdout0)
        self.assertIn("sample-simulator --help", stdout1)
        self.assertIn("sample-simulator --help", stdout2)

        # Test if the same thing happens with an additional run argument (e.g. --no-build)
        return_code0, stdout0, stderr0 = run_ns3('run "sample-simulator --help" --no-build')
        return_code1, stdout1, stderr1 = run_ns3('run sample-simulator --command-template="%s --help" --no-build')
        return_code2, stdout2, stderr2 = run_ns3('run sample-simulator --no-build -- --help')
        self.assertEqual((return_code0, return_code1, return_code2), (0, 0, 0))
        self.assertEqual(stdout0, stdout1)
        self.assertEqual(stdout1, stdout2)
        self.assertEqual(stderr0, stderr1)
        self.assertEqual(stderr1, stderr2)

        # Now collect results for each argument individually
        return_code0, stdout0, stderr0 = run_ns3('run "sample-simulator --PrintGlobals" --verbose')
        return_code1, stdout1, stderr1 = run_ns3('run "sample-simulator --PrintGroups" --verbose')
        return_code2, stdout2, stderr2 = run_ns3('run "sample-simulator --PrintTypeIds" --verbose')

        self.assertEqual((return_code0, return_code1, return_code2), (0, 0, 0))
        self.assertIn("sample-simulator --PrintGlobals", stdout0)
        self.assertIn("sample-simulator --PrintGroups", stdout1)
        self.assertIn("sample-simulator --PrintTypeIds", stdout2)

        # Then check if all the arguments are correctly merged by checking the outputs
        cmd = 'run "sample-simulator --PrintGlobals" --command-template="%s --PrintGroups" --verbose -- --PrintTypeIds'
        return_code, stdout, stderr = run_ns3(cmd)
        self.assertEqual(return_code, 0)

        # The order of the arguments is command template,
        # arguments passed with the target itself
        # and forwarded arguments after the -- separator
        self.assertIn("sample-simulator --PrintGroups --PrintGlobals --PrintTypeIds", stdout)

        # Check if it complains about the missing -- separator
        cmd0 = 'run sample-simulator --command-template="%s " --PrintTypeIds'
        cmd1 = 'run sample-simulator --PrintTypeIds'

        return_code0, stdout0, stderr0 = run_ns3(cmd0)
        return_code1, stdout1, stderr1 = run_ns3(cmd1)
        self.assertEqual((return_code0, return_code1), (1, 1))
        self.assertIn("To forward configuration or runtime options, put them after '--'", stderr0)
        self.assertIn("To forward configuration or runtime options, put them after '--'", stderr1)

    def test_17_RunNoBuildLldb(self):
        """!
        Test if scratch simulator is executed through lldb
        @return None
        """
        if shutil.which("lldb") is None:
            self.skipTest("Missing lldb")

        return_code, stdout, stderr = run_ns3("run scratch-simulator --lldb --verbose --no-build")
        self.assertEqual(return_code, 0)
        self.assertIn("scratch-simulator", stdout)
        self.assertIn("(lldb) target create", stdout)


class NS3QualityControlTestCase(unittest.TestCase):
    """!
    ns-3 tests to control the quality of the repository over time,
    by checking the state of URLs listed and more
    """

    def test_01_CheckForDeadLinksInSources(self):
        """!
        Test if all urls in source files are alive
        @return None
        """

        # Skip this test if Django is not available
        try:
            import django
        except ImportError:
            self.skipTest("Django URL validators are not available")

        # Skip this test if requests library is not available
        try:
            import requests
        except ImportError:
            self.skipTest("Requests library is not available")

        regex = re.compile(r'((http|https)://[^\ \n\)\"\'\}\>\<\]\;\`\\]*)')  # noqa
        skipped_files = []

        whitelisted_urls = {"https://gitlab.com/your-user-name/ns-3-dev",
                            "https://www.nsnam.org/release/ns-allinone-3.31.rc1.tar.bz2",
                            "https://www.nsnam.org/release/ns-allinone-3.X.rcX.tar.bz2",
                            "https://www.nsnam.org/releases/ns-3-x",
                            "https://www.nsnam.org/releases/ns-allinone-3.(x-1",
                            "https://www.nsnam.org/releases/ns-allinone-3.x.tar.bz2",
                            # split due to command-line formatting
                            "https://cmake.org/cmake/help/latest/manual/cmake-",
                            "http://www.ieeeghn.org/wiki/index.php/First-Hand:Digital_Television:_The_",
                            # Dia placeholder xmlns address
                            "http://www.lysator.liu.se/~alla/dia/",
                            # Fails due to bad regex
                            "http://www.ieeeghn.org/wiki/index.php/First-Hand:Digital_Television:_The_Digital_Terrestrial_Television_Broadcasting_(DTTB",
                            "http://en.wikipedia.org/wiki/Namespace_(computer_science",
                            "http://en.wikipedia.org/wiki/Bonobo_(component_model",
                            "http://msdn.microsoft.com/en-us/library/aa365247(v=vs.85",
                            # historical links
                            "http://www.research.att.com/info/kpv/",
                            "http://www.research.att.com/~gsf/",
                            }

        # Scan for all URLs in all files we can parse
        files_and_urls = set()
        unique_urls = set()
        for topdir in ["bindings", "doc", "examples", "src", "utils"]:
            for root, dirs, files in os.walk(topdir):
                # do not parse files in build directories
                if "build" in root or "_static" in root or "source-temp" in root or 'html' in root:
                    continue
                for file in files:
                    # skip svg files
                    if file.endswith(".svg"):
                        continue
                    filepath = os.path.join(root, file)

                    try:
                        with open(filepath, "r") as f:
                            matches = regex.findall(f.read())

                            # Get first group for each match (containing the URL)
                            # and strip final punctuation or commas in matched links
                            # commonly found in the docs
                            urls = list(map(lambda x: x[0][:-1] if x[0][-1] in ".," else x[0], matches))
                    except UnicodeDecodeError:
                        skipped_files.append(filepath)
                        continue

                    # Search for new unique URLs and add keep track of their associated source file
                    for url in set(urls) - unique_urls - whitelisted_urls:
                        unique_urls.add(url)
                        files_and_urls.add((filepath, url))

        # Instantiate the Django URL validator
        from django.core.validators import URLValidator
        from django.core.exceptions import ValidationError
        validate_url = URLValidator()

        # User agent string to make ACM and Elsevier let us check if links to papers are working
        headers = {
            'User-Agent': 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_11_5) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/50.0.2661.102 Safari/537.36'
        }

        def test_file_url(args):
            filepath, url = args
            dead_link_msg = None

            # Skip invalid URLs
            try:
                validate_url(url)
            except ValidationError:
                dead_link_msg = "%s: URL %s, invalid URL" % (filepath, url)

            # Check if valid URLs are alive
            if dead_link_msg is None:
                try:
                    tries = 3
                    while tries > 0:
                        # Not verifying the certificate (verify=False) is potentially dangerous
                        # HEAD checks are not as reliable as GET ones,
                        # in some cases they may return bogus error codes and reasons
                        response = requests.get(url, verify=False, headers=headers)

                        # In case of success and redirection
                        if response.status_code in [200, 301]:
                            dead_link_msg = None
                            break

                        # People use the wrong code, but the reason
                        # can still be correct
                        if response.status_code in [302, 308, 500, 503]:
                            if response.reason.lower() in ['found',
                                                           'moved temporarily',
                                                           'permanent redirect',
                                                           'ok',
                                                           'service temporarily unavailable'
                                                           ]:
                                dead_link_msg = None
                                break
                        # In case it didn't pass in any of the previous tests,
                        # set dead_link_msg with the most recent error and try again
                        dead_link_msg = "%s: URL %s: returned code %d" % (filepath, url, response.status_code)
                        tries -= 1
                except requests.exceptions.InvalidURL:
                    dead_link_msg = "%s: URL %s: invalid URL" % (filepath, url)
                except requests.exceptions.SSLError:
                    dead_link_msg = "%s: URL %s: SSL error" % (filepath, url)
                except requests.exceptions.TooManyRedirects:
                    dead_link_msg = "%s: URL %s: too many redirects" % (filepath, url)
                except Exception as e:
                    try:
                        error_msg = e.args[0].reason.__str__()
                    except AttributeError:
                        error_msg = e.args[0]
                    dead_link_msg = "%s: URL %s: failed with exception: %s" % (filepath, url, error_msg)
            return dead_link_msg

        # Dispatch threads to test multiple URLs concurrently
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=20) as executor:
            dead_links = list(executor.map(test_file_url, list(files_and_urls)))

        # Filter out None entries
        dead_links = list(sorted(filter(lambda x: x is not None, dead_links)))
        self.assertEqual(len(dead_links), 0, msg="\n".join(["Dead links found:", *dead_links]))

    def test_02_MemoryCheckWithSanitizers(self):
        """!
        Test if all tests can be executed without hitting major memory bugs
        @return None
        """
        return_code, stdout, stderr = run_ns3(
            "configure --enable-tests --enable-examples --enable-sanitizers -d optimized")
        self.assertEqual(return_code, 0)

        test_return_code, stdout, stderr = run_program("test.py", "", python=True)

        return_code, stdout, stderr = run_ns3("clean")
        self.assertEqual(return_code, 0)
        self.assertEqual(test_return_code, 0)

    def test_03_CheckImageBrightness(self):
        """!
        Check if images in the docs are above a brightness threshold.
        This should prevent screenshots with dark UI themes.
        @return None
        """
        if shutil.which("convert") is None:
            self.skipTest("Imagemagick was not found")

        from pathlib import Path

        # Scan for images
        image_extensions = ["png", "jpg"]
        images = []
        for extension in image_extensions:
            images += list(Path("./doc").glob("**/figures/*.{ext}".format(ext=extension)))
            images += list(Path("./doc").glob("**/figures/**/*.{ext}".format(ext=extension)))

        # Get the brightness of an image on a scale of 0-100%
        imagemagick_get_image_brightness = \
            'convert {image} -colorspace HSI -channel b -separate +channel -scale 1x1 -format "%[fx:100*u]" info:'

        # We could invert colors of target image to increase its brightness
        # convert source.png -channel RGB -negate target.png
        brightness_threshold = 50
        for image in images:
            brightness = subprocess.check_output(imagemagick_get_image_brightness.format(image=image).split())
            brightness = float(brightness.decode().strip("'\""))
            self.assertGreater(brightness, brightness_threshold,
                               "Image darker than threshold (%d < %d): %s" % (brightness, brightness_threshold, image)
                               )


def main():
    """!
    Main function
    @return None
    """

    test_completeness = {
        "style": [NS3UnusedSourcesTestCase,
                  NS3StyleTestCase,
                  ],
        "build": [NS3CommonSettingsTestCase,
                  NS3ConfigureBuildProfileTestCase,
                  NS3ConfigureTestCase,
                  NS3BuildBaseTestCase,
                  NS3ExpectedUseTestCase,
                  ],
        "complete": [NS3UnusedSourcesTestCase,
                     NS3StyleTestCase,
                     NS3CommonSettingsTestCase,
                     NS3ConfigureBuildProfileTestCase,
                     NS3ConfigureTestCase,
                     NS3BuildBaseTestCase,
                     NS3ExpectedUseTestCase,
                     NS3QualityControlTestCase,
                     ],
        "extras": [NS3DependenciesTestCase,
                   ]
    }

    import argparse

    parser = argparse.ArgumentParser("Test suite for the ns-3 buildsystem")
    parser.add_argument("-c", "--completeness",
                        choices=test_completeness.keys(),
                        default="complete")
    parser.add_argument("-tn", "--test-name",
                        action="store",
                        default=None,
                        type=str)
    parser.add_argument("-q", "--quiet",
                        action="store_true",
                        default=False)
    args = parser.parse_args(sys.argv[1:])

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Put tests cases in order
    for testCase in test_completeness[args.completeness]:
        suite.addTests(loader.loadTestsFromTestCase(testCase))

    # Filter tests by name
    if args.test_name:
        # Generate a dictionary of test names and their objects
        tests = dict(map(lambda x: (x._testMethodName, x), suite._tests))

        tests_to_run = set(map(lambda x: x if args.test_name in x else None, tests.keys()))
        tests_to_remove = set(tests) - set(tests_to_run)
        for test_to_remove in tests_to_remove:
            suite._tests.remove(tests[test_to_remove])

    # Before running, check if ns3rc exists and save it
    ns3rc_script_bak = ns3rc_script + ".bak"
    if os.path.exists(ns3rc_script) and not os.path.exists(ns3rc_script_bak):
        shutil.move(ns3rc_script, ns3rc_script_bak)

    # Run tests and fail as fast as possible
    runner = unittest.TextTestRunner(failfast=True, verbosity=1 if args.quiet else 2)
    result = runner.run(suite)

    # After completing the tests successfully, restore the ns3rc file
    if os.path.exists(ns3rc_script_bak):
        shutil.move(ns3rc_script_bak, ns3rc_script)


if __name__ == '__main__':
    main()
