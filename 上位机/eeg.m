% clear;
clc;
obj=serial('COM4');%����һ�����ڶ���
fopen(obj);%�򿪴���
set(obj,'BaudRate', 115200,'timeout',600,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');%���ô��ڲ�����time? out ����Ҫ�����һ��ȽϺá�
N=30;%��N������
data=fread(obj,N)';%��ȡ����,����Ϊ��������ת�ó�������
fclose(obj);%����رգ����򱨴�

%��������
a=sum(data(1,:))/N;%��ֵ
b=std2(data);%������

%xlswrite('20160310','A1:AD1',data);? %д��excel��� A1��AD1ÿ�μ�һ�ֶ����ġ�

%��ͼ
figure
hold on;box on;
plot(1:N,data(1,:),'-o');
xlabel(['average=',num2str(a),'? ','\sigma=',num2str(b)]);

% figure
% hold on;