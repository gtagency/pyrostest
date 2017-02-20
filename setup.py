from setuptools import setup

required = [
    'psutil==5.1.3'
    ]

test = []

setup(
    name='pyrostest',
    version='0.0.2',
    description='The most lit ros testing framework',
    packages=['pyrostest'],
    install_requires=required,
    license='MIT',
    classifiers=[
        ],
    )


