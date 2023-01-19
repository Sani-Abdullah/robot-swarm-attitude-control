import config as cf


def halting_disabled(agent):
    def inner(func):
        if cf.HALTING in agent.states:
            func()

    return inner
