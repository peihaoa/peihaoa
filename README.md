自闭症谱系障碍儿童声学特征滤波识别诊断系统

我们通过基于Minspore框架的深度学习进行训练优化模型，最终形成一套准确率高的模型，为自闭症儿童的识别提供依据，检测后系统会显示反馈结果，医院医师可以快速的获取诊断结果。

我们开发的自闭症儿童声学特征滤波识别诊断系统，帮助医生辅助诊断自闭症儿童。医师通过我们的设备收集患者的声音并上传服务器，在服务器上经过Berouti谱减法降噪，使用OpenSmile工具包、Librosa库提取声学特征绘制MFCC图像，并通过我们使用Mindspore框架训练好的模型，得到诊断结果，并且传输到医师手机端，反馈给医生，最后由医生给出专业的建议以及诊断，我们的产品会将医生的诊断意见反馈给患者家长。

该源代码为我们硬件部分的代码,功能主要为蓝牙控制,音频采集和音频播放