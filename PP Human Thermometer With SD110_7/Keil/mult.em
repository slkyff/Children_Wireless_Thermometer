macro MultiLineComment()

{

    hwnd = GetCurrentWnd()

    selection = GetWndSel(hwnd)

    LnFirst =GetWndSelLnFirst(hwnd)      //ȡ�����к�

    LnLast =GetWndSelLnLast(hwnd)      //ȡĩ���к�

    hbuf = GetCurrentBuf()

 

    if(GetBufLine(hbuf, 0) =="//magic-number:tph85666031"){

        stop

    }

 

    Ln = Lnfirst

    buf = GetBufLine(hbuf, Ln)

    len = strlen(buf)

 

    while(Ln <= Lnlast) {

        buf = GetBufLine(hbuf, Ln)  //ȡLn��Ӧ����

        if(buf ==""){                   //��������

            Ln = Ln + 1

            continue

        }

 

        if(StrMid(buf, 0, 1) == "/"){       //��Ҫȡ��ע��,��ֹֻ�е��ַ�����

            if(StrMid(buf, 1, 2) == "/"){

                PutBufLine(hbuf, Ln, StrMid(buf, 2, Strlen(buf)))

            }

        }

 

        if(StrMid(buf,0,1) !="/"){          //��Ҫ����ע��

            PutBufLine(hbuf, Ln, Cat("//", buf))

        }

        Ln = Ln + 1

    }

 

    SetWndSel(hwnd, selection)

}

