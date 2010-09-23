from os.path import abspath, dirname
from ompl import dll_loader
from ompl.geometric import *
dll_loader('ompl_app', dirname(abspath(__file__)))
from _app import *
