from battmodel import BattModel
from estimator import BattEstimator

from visual import *
from visual.graph import *

model = BattModel()
estimator = BattEstimator()

volt_plot = gcurve(color=color.red)
curr_plot = gcurve(color=color.green)
est_volt_plot = gcurve(color=color.blue)
est_curr_plot = gcurve(color=color.blue)

dt = 1.0/60.0
t = 0
while(True):
    rate(60)
    v_meas = model.voltage_meas(t)
    i_meas = model.current_meas(t)
    #print t,i_meas
    meas = [v_meas, i_meas]
    estimator.predict(dt)
    estimator.update(meas)
    volt_plot.plot(pos=(t,model.voltage_meas(t)))
    curr_plot.plot(pos=(t,model.current_meas(t)))
    est_volt_plot.plot(pos=(t,estimator.resting_voltage_estimate()))
    est_curr_plot.plot(pos=(t,estimator.current_estimate()))
    #print estimator.resistance_estimate()
    t+=dt
