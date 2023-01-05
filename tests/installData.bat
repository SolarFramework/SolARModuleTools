@echo off
:: Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
powershell Expand-Archive fbow_voc.zip -DestinationPath .\data -F
del fbow_voc.zip

:: Download maps
curl https://repository.solarframework.org/generic/captures/singleRGB/TUM/freiburg3_long_office_household.zip -L -o map.zip
powershell Expand-Archive map.zip -DestinationPath .\data -F
del map.zip

:: Install required external modules
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug