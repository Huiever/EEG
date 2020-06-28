function SerialFuc()        % 创建函数  
  delete(instrfindall)      % 关闭串口，此句十分重要，下篇再详细解释  
  global s a fid;% 全局变量，供串口中断函数使用  
  a=[];
  format short g
  fid = fopen('ads_data1.txt','a+');
  fclose('all');
  delete('ads_data1.txt');
  fid = fopen('ads_data1.txt','a+');
  s = serial('com5');       %使用默认设置创建串口s  
  set(s,'BaudRate',115200);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
  set(s,'InputBufferSize',31);%%%设置输入缓冲区大小为1
  set(s,'BytesAvailableFcnMode','byte'); %设置中断触发方式  
  set(s,'BytesAvailableFcnCount',31); %  
  s.BytesAvailableFcn =@ReceiveCallback;       % 定义中断响应函数对象  
  fopen(s);                 %打开串口  
  global i;
  i = 1;
%  fclose(s);            % 关闭串口
%  delete(s);           % 释放串口对象占用的内存空间,
%  clear s;  
end
function ReceiveCallback( obj,event)     %创建中断响应函数  
    global s a fid; 
    str = fread(s); %读取数据
%   hex = compose("%X",str)
    data = zeros(1,8);
    sign_head1 = hex2dec('0D');
    sign_head2 = hex2dec('0A');
    sign_finish1 = hex2dec('0A');
    sign_finish2 = hex2dec('0D');
    a = [];
    a = [a;str];
    j = 1;
    while (~isempty(a))
         if j>size(a,1)
           break;
         end
         if a(j)==sign_head1 && a(j+1) == sign_head2 
            if (j + 31 - 1) > size(a,1) 
                break;
            end
            index_start = j + 2;
            index_finish= index_start + 31 - 2 - 1;
            pack = a(index_start:index_finish);
            if ~isempty(pack) && pack(28) == sign_finish1 && pack(29)== sign_finish2
                for m = 1:8                        
                    n = m * 3 + 1;
                    HEX = bitshift(pack(n),16) + bitshift(pack(n+1),8) + pack(n+2);
                    if bitand(HEX ,hex2dec('800000'))
                        data(m) = (16777216 - HEX) * (-4500000)/8388607;
                    else
                        data(m) = HEX * (4500000)/8388607;                        
                    end
                end
                fprintf(fid,'%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\n',data);
                
                a(1:index_finish)=[];
                j = 1;
            else
                j = j + 1;
            end
         else
             j = j + 1;
         end
    end
end