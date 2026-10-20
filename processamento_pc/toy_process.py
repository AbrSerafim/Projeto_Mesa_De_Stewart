from Timer import PipelineTimer

timer = PipelineTimer()

# 1. Capture image
with timer.measure("Image capture"):
    pass

timer.report()