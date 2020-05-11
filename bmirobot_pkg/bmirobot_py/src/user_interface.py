#!/usr/bin/env python
"""
User interface about joint angle control
Author: Zheng Haosi
Date: 2020.2.21
"""
import tkinter as tk


window = tk.Tk()
window.title('Joint angle UI')
window.geometry('500x300')

text_list = ['joint 1', 'joint 2', 'joint 3', 'joint 4', 'joint 5', 'joint 6', 'joint 7']
for i in range(len(text_list)):
    tk.Label(window, text=text_list[i]).grid(row=i, column=1, padx=10, pady=10)
    tk.Entry(window, width=10).grid(row=i, column=2, padx=10, pady=10)

# label_joint1 = tk.Label(window,
#     text='Joint1',    # 标签的文字
#        # 背景颜色
#     font=('Arial', 12),     # 字体和字体大小
#     width=15, height=2  # 标签长宽
#     )
# label_joint1.pack()    # 固定窗口位置


button_g = tk.Button(window,
    text='grasp',      # 显示在按钮上的文字
    width=15, height=2,
    command=None)     # 点击按钮式执行的命令
button_g.grid(row=1, column=7)    # 按钮位置

button_r = tk.Button(window,
    text='return',      # 显示在按钮上的文字
    width=15, height=2,
    command=None)     # 点击按钮式执行的命令
button_r.grid(row=3, column=7)    # 按钮位置

window.mainloop()