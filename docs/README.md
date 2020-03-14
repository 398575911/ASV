# ASV 说明文档


### 编译模板

#### 在 Linux 与 macOS 上编译

发行包中配套了 `Makefile` 文件。可以使用 GNU Make 工具调用 `latexmk` 程序，自动完成模板的多轮编译。`Makefile` 同时也提供了一些额外的实用功能。

```bash
make thesis.pdf     # 编译模板，生成 thesis.pdf
make clean          # 清理文件（不包括 thesis.pdf）
make cleanall       # 清理文件（包括 thesis.pdf）
make wordcount      # 字数统计
```

若需要生成用于提交盲审的论文（隐去作者、导师等信息），可在模版类选项中添加 `review`。

```latex
\documentclass[degree=master, zihao=-4, review]{sjtuthesis}
```

若需要在生成的论文中添加「原创性声明」和「版权使用授权书」的签名扫描件，可将 PDF 格式扫描件的路径作为参数传递给 `\makeDeclareOriginality` 和 `\makeDeclareAuthorization` 命令。

```latex
\makeDeclareOriginality[pdf/originality.pdf]
\makeDeclareAuthorization[pdf/authorization.pdf]
```

#### 在 Windows 上编译

双击 `compile.bat` 即可完成编译过程，生成 `thesis.pdf`。

#### 问题诊断

编译失败时，可以尝试手动逐次编译，定位故障。

```bash
xelatex -no-pdf thesis
biber --debug thesis
xelatex thesis
xelatex thesis
```


