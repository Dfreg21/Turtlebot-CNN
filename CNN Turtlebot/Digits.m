clear all
close all
clc

%% Cargar y explorar datos de imágenes
digitDatasetPath = fullfile(matlabroot,'toolbox','nnet','nndemos','nndatasets','DigitDataset');
imds = imageDatastore(digitDatasetPath,'IncludeSubfolders',true,'LabelSource','foldernames');

N = size(imds.Files,1);

figure;
perm = randperm(N,20);
for i = 1:20
    subplot(4,5,i);
    imshow(imds.Files{perm(i)});
end

%% Datos utiles
labelCount = countEachLabel(imds);
disp(labelCount)

img = readimage(imds,1);
img_size = size(img);
disp(img_size)

%% Especificar los conjuntos de datos de entrenamiento y de validación
numTrainFiles = 0.8;
[imdsTrain,imdsValidation] = splitEachLabel(imds,numTrainFiles,'randomize');

%% Definir la arquitectura de red
layers = [imageInputLayer([img_size 1])

          convolution2dLayer(3,8,'Padding','same')
          batchNormalizationLayer
          reluLayer
    
          maxPooling2dLayer(2,'Stride',2)
          convolution2dLayer(3,16,'Padding','same')
          batchNormalizationLayer
          reluLayer
    
          maxPooling2dLayer(2,'Stride',2)
          convolution2dLayer(3,32,'Padding','same')
          batchNormalizationLayer
          reluLayer
    
          fullyConnectedLayer(10)
          softmaxLayer
          classificationLayer];

%% Especificar las opciones de entrenamiento
options = trainingOptions('adam', ...
    'InitialLearnRate',0.01, ...
    'MaxEpochs',16, ...
    'Shuffle','every-epoch', ...
    'ValidationData',imdsValidation, ...
    'ValidationFrequency',30, ...
    'Verbose',true, ...
    'Plots','training-progress');

%% Entrenar la red con datos de entrenamiento
net = trainNetwork(imdsTrain,layers,options);
save('cnn_net_digits','net')

%% Clasificar imágenes de validación y calcular la precisión
YPred = classify(net,imdsValidation);
YValidation = imdsValidation.Labels;

accuracy = sum(YPred == YValidation)/numel(YValidation);
disp(accuracy)

figure
confusionchart(YValidation,YPred)

