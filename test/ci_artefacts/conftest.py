import os


def pytest_configure(config):
    """junit output path for the Artefacts CI scenarios (artefacts.yaml), written to its upload directory"""
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    if upload_dir and not config.option.xmlpath:
        config.option.xmlpath = os.path.join(upload_dir, "tests_junit.xml")
