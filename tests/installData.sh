# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
unzip -o fbow_voc.zip -d ./data 
rm fbow_voc.zip

# Download AR device capture
curl https://artifact.b-com.com/solar-generic-local/captures/singleRGB/TUM/freiburg3_long_office_household.zip -L -o map.zip
unzip -o map.zip -d ./data
rm map.zip

# Install required packagedependencies.txt
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

