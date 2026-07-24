import os


def pytest_configure(config):
    """Write a JUnit XML report into the Artefacts upload dir when running
    under an Artefacts scenario (equivalent to passing --junit-xml)."""
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    if upload_dir and not config.option.xmlpath:
        config.option.xmlpath = os.path.join(upload_dir, "tests_junit.xml")
