profile1 = readtable('~/RBE3001_A23_Team22/baseprofile1.csv');
profile2 = readtable('~/RBE3001_A23_Team22/baseprofile2.csv');
profile3 = readtable('~/RBE3001_A23_Team22/baseprofile3.csv');


plot(profile1.Var1(1:313,1))    
hold on
plot(profile2.Var1(1:313,1))
plot(profile3.Var1(1:313,1))

title('Base Joint Consistency')
xlabel('Time (ms)')
ylabel('Joint Movement in Degrees')

legend('Profile 1', 'Profile 2', 'Profile 3')
hold off