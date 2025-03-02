from setuptools import setup, find_packages

setup(
    name='ros2_sdk',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'msgpack>=1.0.2',
        'rx>=3.2.0',
    ],
    author='Johannes Mesner',
    author_email='johannes.mesner@tum.de',
    description='Python SDK for ROS2 API with reactive streams',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    license='MIT',
    url='https://github.com/mesnero/ros2_sdk',  # Update this URL to your repository
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
    ],
)
