% clear;
clc;
obj=serial('COM4');%建立一个串口对象
fopen(obj);%打开串口
set(obj,'BaudRate', 115200,'timeout',600,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');%设置串口参数，time? out 很重要，设大一点比较好。
N=30;%读N个数据
data=fread(obj,N)';%读取串口,读出为列向量，转置成行向量
fclose(obj);%读完关闭，否则报错

%处理数据
a=sum(data(1,:))/N;%均值
b=std2(data);%均方差

%xlswrite('20160310','A1:AD1',data);? %写入excel表格 A1：AD1每次加一手动更改。

%绘图
figure
hold on;box on;
plot(1:N,data(1,:),'-o');
xlabel(['average=',num2str(a),'? ','\sigma=',num2str(b)]);

% figure
% hold on;