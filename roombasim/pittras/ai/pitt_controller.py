'''
pitt_controller.py
'''

from roombasim.ai import Controller

class PittController(Controller):

    def update(self, delta, elapsed):
        print("update", delta)
