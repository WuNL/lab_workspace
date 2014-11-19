

#include <QThread>
#include <QObject>

class t1:public QThread
{
    Q_OBJECT
public:
    t1();
    ~t1();
	void run();
	void messageemit();
	void messageemit_people();
	bool is_save(bool save);
public Q_SLOTS:
    void showdata();
    void showpeople();
Q_SIGNALS:
    void update();
    void update_people();
private:
	bool is_save_;
	double h2p_a_,h2p_b_,h2p_c_,h2p_d_,h2p_e_,h2p_f_,h2p_g_,h2p_h_,h2p_i_;//hokuyo激光到云台变换矩阵R中的值
};
