function result = generate_uplink(M, N)
    Data = readmatrix('/tmp/Hydra/matlab_input.txt');
    mimoChan = comm.MIMOChannel('SpatialCorrelationSpecification', 'None', 'NumTransmitAntennas', M, 'NumReceiveAntennas', N);
    output_data = mimoChan(Data);
    writematrix(output_data, '/tmp/Hydra/matlab_output.txt', 'delimiter', ' ');
    result = 0
end