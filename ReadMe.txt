SimuHEV BMWi3 Car Model Simulation CourseWork
BY:
Pawan Kumar Dhakal
Email-- UO286375@uniovi.es



%Read "Final_Simulation_BMWi3_Report" to get the general idea on the vehicle model
and the modelling methodology and design steps followed%

STEPS To RUN the simulation Models:

%%MECHANICAL MODEL%%
1. Run the "parameters_BMWi3.m" matlab script
2. Open "Final_Simulation_BMWi3_Mechanical_Model.slx" model
3. Run the model and check the scopes in Measurement Area for vehicle performance

%%AVERAGE MODEL%
1. Open the "parameters_BMWi3.m" matlab script
2. In Boost converter controller section, comment the switching model bandwidths for current and 
voltage controller if not already commented and uncommnet the same for average model
3. Run the matlab Script
4. Open "Final_Simulation_BMWi3_Average_Model.slx" model
5. Run the model and check scopes in Measurement Area for vehicle performance, DC-DC converter
performance and other paramters


%%SWITCHING MODEL%%
1.Open the "parameters_BMWi3.m" matlab script
2.In Boost converter controller section, comment the average model bandwidths for current and
voltage controller if not already commented and uncommnet the same for switching model.
3.Run the matlab Script
4.Open "Final_Simulation_BMWi3_Switching_Model.slx" model
5.Run the model and check scopes in Measurement Area for vehicle performance, DC-DC converter
performance and other paramters
 
