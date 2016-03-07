% Very simple example to verify that your mexmoos is installed correctly

%% Clear any previous connections to MOOS
clear mexmoos;

%% Connect to MOOS
unique_num = num2str(int32(rand*1e7));
host = '192.168.0.106';
client = ['`\_(^_^)_/`' ''];
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client, 'SERVERPORT','9000');
pause(1.0); % give mexmoos a chance to connect (important!)

%% Register (i.e., "listen") for messages on a specific MOOS channel
channel_name = ['CdtExampleChannel' unique_num]; %%CdtExampleChannel == Variable Name
mexmoos('REGISTER', channel_name, 0.0);

%% Send a message over the channel
disp(['Sending message over MOOS with unique number: ' unique_num]);
mexmoos('NOTIFY', channel_name, message_to_send);


for i = 1:100
    message_to_send = [num2str(i) 'Im not your buddy pal'];
    mexmoos('NOTIFY', channel_name, message_to_send);
    pause(0.5);
end

%% See if we can read the same message back from MOOS
disp('Reading message from MOOS');
pause(0.1);
moos_mailbox = mexmoos('FETCH');
msg_idx = find(strcmp({moos_mailbox.KEY}, channel_name));
for idx = msg_idx
    disp(['Received Message:  "' moos_mailbox(idx).STR '"']);
end
if isempty(msg_idx)
    error('Did not receive message from MOOS!');
end

%% Cleanup
mexmoos('CLOSE');