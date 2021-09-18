class xbox():
    """
    Left Stick:
        Left -> Right   - Axis 0
        Up   -> Down    - Axis 1
    Right Stick:
        Left -> Right   - Axis 3
        Up   -> Down    - Axis 4
    Left Trigger:
        Out -> In       - Axis 2
    Right Trigger:
        Out -> In       - Axis 5
    Buttons:
        A Button        - Button 0
        B Button        - Button 1
        X Button        - Button 2
        Y Button        - Button 3
        Left Bumper     - Button 4
        Right Bumper    - Button 5
        Back Button     - Button 6
        Start Button    - Button 7
        L. Stick In     - Button 8
        R. Stick In     - Button 9
        Guide Button    - Button 10
    Hat/D-pad:
        Down -> Up      - Y Axis
        Left -> Right   - X Axis
    """
    LT=2;                                 RT=5
    LB=4;                                 RB=5

    ...;                                  Y=3
    LLR=0;      BB=6; GB=10; SB=7;      X=2; B=1
    LUD=1;                                A=0
    LSI=8

    ...; HU=[1,0];                     RLR=3
    HL=[0,-1]; HR=[0,1];               RUD=4
    ...; HD=[-1,0];                    RSI=9
