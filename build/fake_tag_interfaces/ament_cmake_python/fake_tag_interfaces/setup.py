from setuptools import find_packages
from setuptools import setup

setup(
    name='fake_tag_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('fake_tag_interfaces', 'fake_tag_interfaces.*')),
)
