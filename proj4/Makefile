all: obj bin teststrutils teststrdatasource teststrdatasink testdsv testxml testcsvbs testosm testcsvbsindex run

obj:
	mkdir -p obj

bin:
	mkdir -p bin

obj/StringUtils.o: src/StringUtils.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/StringUtils.o -c src/StringUtils.cpp

obj/StringUtilsTest.o: testsrc/StringUtilsTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/StringUtilsTest.o -c testsrc/StringUtilsTest.cpp

obj/StringDataSource.o: src/StringDataSource.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/StringDataSource.o -c src/StringDataSource.cpp

obj/StringDataSourceTest.o: testsrc/StringDataSourceTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/StringDataSourceTest.o -c testsrc/StringDataSourceTest.cpp

obj/StringDataSink.o: src/StringDataSink.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/StringDataSink.o -c src/StringDataSink.cpp

obj/StringDataSinkTest.o: testsrc/StringDataSinkTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/StringDataSinkTest.o -c testsrc/StringDataSinkTest.cpp

obj/DSVReader.o: src/DSVReader.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/DSVReader.o -c src/DSVReader.cpp

obj/DSVWriter.o: src/DSVWriter.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/DSVWriter.o -c src/DSVWriter.cpp

obj/DSVTest.o: testsrc/DSVTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/DSVTest.o -c testsrc/DSVTest.cpp

obj/XMLReader.o: src/XMLReader.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/XMLReader.o -c src/XMLReader.cpp 

obj/XMLWriter.o: src/XMLWriter.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/XMLWriter.o -c src/XMLWriter.cpp

obj/XMLTest.o: testsrc/XMLTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/XMLTest.o -c testsrc/XMLTest.cpp

obj/OpenStreetMap.o: src/OpenStreetMap.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/OpenStreetMap.o -c src/OpenStreetMap.cpp

obj/OpenStreetMapTest.o: testsrc/OpenStreetMapTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/OpenStreetMapTest.o -c testsrc/OpenStreetMapTest.cpp

obj/CSVBusSystem.o: src/CSVBusSystem.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/CSVBusSystem.o -c src/CSVBusSystem.cpp

obj/CSVBusSystemTest.o: testsrc/CSVBusSystemTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/CSVBusSystemTest.o -c testsrc/CSVBusSystemTest.cpp

obj/CSVBusSystemIndexer.o: src/BusSystemIndexer.cpp | obj
	g++ -std=c++17 -g -Iinclude -o obj/CSVBusSystemIndexer.o -c src/BusSystemIndexer.cpp

obj/CSVBusSystemIndexerTest.o: testsrc/CSVBusSystemIndexerTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/CSVBusSystemIndexerTest.o -c testsrc/CSVBusSystemIndexerTest.cpp

# obj/CTransportationPlannerCommandLine.o: src/CTransportationPlannerCommandLine.cpp | obj
# 	g++ -std=c++17 -g -Iinclude -o obj/CTransportationPlannerCommandLine.o -c src/CTransportationPlannerCommandLine.cpp

obj/CSVOSMTransportationPlannerTest.o: testsrc/CSVOSMTransportationPlannerTest.cpp | obj
	g++ -Iinclude -g -std=c++17 -o obj/CSVOSMTransportationPlannerTest.o -c testsrc/CSVOSMTransportationPlannerTest.cpp

teststrutils: obj/StringUtils.o obj/StringUtilsTest.o | bin
	g++ -g obj/StringUtils.o obj/StringUtilsTest.o -o bin/teststrutils -lgtest -lgtest_main -lexpat

teststrdatasource: obj/StringDataSource.o obj/StringDataSourceTest.o | bin
	g++ -g obj/StringDataSource.o obj/StringDataSourceTest.o -o bin/teststrdatasource -lgtest -lgtest_main -lexpat

teststrdatasink: obj/StringDataSink.o obj/StringDataSinkTest.o | bin
	g++ -g obj/StringDataSink.o obj/StringDataSinkTest.o -o bin/teststrdatasink -lgtest -lgtest_main -lexpat

testdsv: obj/DSVReader.o obj/DSVWriter.o obj/DSVTest.o obj/StringUtils.o obj/StringDataSource.o obj/StringDataSink.o | bin
	g++ -g obj/DSVReader.o obj/DSVWriter.o obj/DSVTest.o obj/StringUtils.o obj/StringDataSource.o obj/StringDataSink.o -o bin/testdsv -lgtest -lgtest_main

testxml: obj/XMLReader.o obj/XMLWriter.o obj/XMLTest.o obj/StringUtils.o obj/StringDataSource.o obj/StringDataSink.o | bin
	g++ -g obj/XMLReader.o obj/XMLWriter.o obj/XMLTest.o obj/StringUtils.o obj/StringDataSource.o obj/StringDataSink.o -o bin/testxml -lgtest -lgtest_main -lexpat

testcsvbs: obj/CSVBusSystem.o obj/CSVBusSystemTest.o obj/DSVReader.o obj/StringDataSource.o obj/StringUtils.o | bin
	g++ -g obj/CSVBusSystem.o obj/CSVBusSystemTest.o obj/DSVReader.o obj/StringDataSource.o obj/StringUtils.o -o bin/testcsvbs -lgtest -lgtest_main

testosm: obj/OpenStreetMap.o obj/OpenStreetMapTest.o obj/XMLReader.o obj/StringUtils.o obj/StringDataSource.o | bin
	g++ -g obj/OpenStreetMap.o obj/OpenStreetMapTest.o obj/XMLReader.o obj/StringUtils.o obj/StringDataSource.o -o bin/testosm -lgtest -lgtest_main -lexpat

testcsvbsindex: obj/CSVBusSystemIndexer.o obj/CSVBusSystemIndexerTest.o obj/CSVBusSystem.o obj/DSVReader.o obj/StringDataSource.o obj/StringUtils.o | bin
	g++ -g obj/CSVBusSystemIndexer.o obj/CSVBusSystemIndexerTest.o obj/CSVBusSystem.o obj/DSVReader.o obj/StringDataSource.o obj/StringUtils.o -o bin/testcsvbsindex -lgtest -lgtest_main

# testcsvosmtp: obj/CSVBusSystem.o obj/CSVOSMTransportationPlannerTest.o obj/DSVReader.o obj/StringDataSource.o obj/StringUtils.o obj/OpenStreetMap.o obj/XMLReader.o obj/CSVBusSystemIndexer.o | bin
# 	g++ -g obj/CSVBusSystem.o obj/CSVOSMTransportationPlannerTest.o obj/DSVReader.o obj/StringDataSource.o obj/StringUtils.o obj/OpenStreetMap.o obj/XMLReader.o obj/CSVBusSystemIndexer.o -o bin/testcsvosmtp -lgtest -lgtest_main -lexpat

clean:
	rm -rf obj bin
	rm -f teststrutils teststrdatasource teststrdatasink testdsv testxml testcsvbs testosm

run: teststrutils teststrdatasource teststrdatasink testdsv testxml testcsvbs testosm testcsvbsindex 
# testcsvbsindex testcsvosmtp
	./bin/teststrutils
	./bin/teststrdatasource
	./bin/teststrdatasink
	./bin/testdsv
	./bin/testxml
	./bin/testcsvbs
	./bin/testosm
	./bin/testcsvbsindex
# ./bin/testcsvbsindex
# ./bin/testcsvosmtp