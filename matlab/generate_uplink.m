function result = generate_uplink(M, N)
    fidi = fopen('/tmp/Hydra/matlab_input.txt', 'r');
    Data = textscan(fidi, '%f%f');
    Data = cell2mat(Data);
    mimoChan = comm.MIMOChannel('SpatialCorrelationSpecification', 'None', 'NumTransmitAntennas', M, 'NumReceiveAntennas', N);
    output_data = mimoChan(Data);
    writematrix(output_data, '/tmp/Hydra/matlab_output.txt', 'delimiter', ' ');
    result = 0
end