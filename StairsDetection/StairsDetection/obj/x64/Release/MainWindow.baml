   M S B A M L   `   `   `  ���� J  FStairsDetection, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null1clr-namespace:StairsDetectionStairsDetection       StairsDetection.MainWindow   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     /localclr-namespace:StairsDetection     -     � Title$  StairsDetection�� � SizeToContent$ WidthAndHeight=�.��� $��10 10 10 10q� � ColumnDefinitions �� $	��522 ��� $	��522 ��� $	��522 � � RowDefinitions �� $	��434 ��� $	��434 ��� $��30 �.+��� -    ��Name$	 ths��$��2��$��0��$��2��$
��Left=�$��Center=��� -   $ distance��$��2��$��2��$��Center=�$��Center=��� -    ��Name$ image_depth��$��0��$��0��$	��512��$	��424��$��5 5 5 5q�$
��Left=�$	��Top=��� -   $ image_stairs��$��2��$��0��$	��512��$	��424��$��5 5 5 5q��� -   $ image_canny��$��0��$��1��$	��512��$	��424��$��5 5 5 5q�$
��Left=�$��Bottom=��� -   $ image_hough��$��2��$��1��$	��512��$	��424��$��5 5 5 5q�$
��Left=��� -   $ image_stepCandidate��$��1��$��0��$	��512��$	��424��$��5 5 5 5q�$
��Left=��� -	   $ image_combinedLine��$��1��$��1��$	��512��$	��424��$��5 5 5 5q�$
��Left=��� -
   $ image_color��$��3��$��1��$	��512��$	��424��$��5 5 5 5q�$
��Left=��� -    ��Name$ CanvasPoint��$��1��$��0��$	��512��$	��424��$
��Left=��� $�� 保存方式選択��$	��130��$	��100��$
��Left=�$	��Top=�
�������$��1��$��192,100,0,0q�$��1��.����  �� Orientation$ Vertical=�$��10q�.+�	� -    	�Name$ IsSuccesion��$��個別撮影��	 d� 	IsChecked$
	 TrueY�	� $��連続撮影��