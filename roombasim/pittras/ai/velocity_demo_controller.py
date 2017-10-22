'''
velocity_demo_controller.py
'''

from roombasim.ai import Controller

def velocity_task_completion_callback(status):
    '''
    Callback for Velocity task completion.
    '''
    print("Velocity task completed with", status)

class VelocityDemoController(Controller):
    '''
    A demo controller tests VelocityTask.
    '''

    def setup(self):
        self.task_controller.switch_task(
            'VelocityTask',
            callback=velocity_task_completion_callback,
            target=[5, 5, 1]
        )
