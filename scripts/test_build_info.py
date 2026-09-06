"""Unit tests for build_info.py. Run with:

    python -m unittest discover -s scripts -p 'test_*.py'
"""
import pathlib
import subprocess
import tempfile
import unittest

import build_info


def make_repo(path: pathlib.Path) -> None:
    """Create a throwaway git repo with one commit."""
    subprocess.run(["git", "init", "-q"], cwd=path, check=True)
    subprocess.run(["git", "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-q", "--allow-empty",
                    "-m", "init"], cwd=path, check=True)


class BuildInfoTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = pathlib.Path(self.tmp.name)
        make_repo(self.repo)
        self.out = self.repo / "gen" / "build_info.hh"

    def tearDown(self):
        self.tmp.cleanup()

    def test_writes_defines_for_a_clean_repo(self):
        self.assertTrue(build_info.write_build_info(self.repo, self.out))
        text = self.out.read_text()
        info = build_info.describe(self.repo)
        self.assertIn(f'#define BUILD_GIT_SHA "{info["sha"]}"', text)
        self.assertIn("#define BUILD_GIT_DIRTY 0", text)
        self.assertIn(f'#define BUILD_VERSION "{info["sha"][:7]}', text)  # describe --always -> short hash
        self.assertNotIn("dirty", text)

    def test_marks_uncommitted_tracked_changes_dirty(self):
        tracked = self.repo / "file.txt"
        tracked.write_text("a")
        subprocess.run(["git", "add", "file.txt"], cwd=self.repo, check=True)
        subprocess.run(["git", "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-q", "-m", "add"],
                       cwd=self.repo, check=True)
        tracked.write_text("b")
        build_info.write_build_info(self.repo, self.out)
        text = self.out.read_text()
        self.assertIn("#define BUILD_GIT_DIRTY 1", text)
        self.assertIn("-dirty", text)

    def test_uses_tags_when_present(self):
        subprocess.run(["git", "tag", "v1.0"], cwd=self.repo, check=True)
        build_info.write_build_info(self.repo, self.out)
        self.assertIn('#define BUILD_VERSION "v1.0"', self.out.read_text())

    def test_does_not_rewrite_an_unchanged_header(self):
        self.assertTrue(build_info.write_build_info(self.repo, self.out))
        mtime = self.out.stat().st_mtime_ns
        self.assertFalse(build_info.write_build_info(self.repo, self.out))
        self.assertEqual(mtime, self.out.stat().st_mtime_ns)

    def test_falls_back_outside_a_checkout(self):
        with tempfile.TemporaryDirectory() as plain:
            info = build_info.describe(pathlib.Path(plain))
        self.assertEqual(info, {"sha": "unknown", "dirty": False, "version": "unknown"})

    def test_cli_writes_to_the_requested_path(self):
        self.assertEqual(build_info.main(["--repo", str(self.repo), "-o", str(self.out)]), 0)
        self.assertTrue(self.out.exists())


if __name__ == "__main__":
    unittest.main()
