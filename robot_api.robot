*** Settings ***
Documentation
...  == Author ==
...  Manish Subedi
...
...  == commands for controlling robot ==

Library           Collections
Library           api_client.py

*** Variables ***
${UNAME}          user
${PWD}            pass
${TOKEN}          None

*** Keywords ***
_Authenticate        
    [Arguments]  ${robot_name}
    Set Test Variable  ${BASE_URL}  http://${robot_name}.test.com:8000
    ${response}  api_client.Authenticate   ${BASE_URL}/token   ${UNAME}   ${PWD}
    Log   ${response}

Robot Init  
    [Arguments]  ${robot_name}
    Authenticate  ${robot_name}
    ${response}  api_client.GetRequest    ${BASE_URL}    start
    Log  ${response}  

Robot Disable
    [Arguments]  ${robot_name}
    Authenticate  ${robot_name}
    ${response}  api_client.GetRequest    ${BASE_URL}   stop    
    Log  ${response}