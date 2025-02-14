using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using UnityEngine;

public class procedure_control_pcd : MonoBehaviour
{
    public target_sub target_sub;
    public pcd_grasp_agent agent;
    public rosJS_control js_init;
    public jaw_state_pub jaw;
    public JointState_Pub JS_pub;
    public joint_state_sub JS_sub;
    public after_reach_pcd after_reach;

    public List<float> JS= new List<float> {0f,0f,0f,0f,0f,0f};
    // Start is called before the first frame update
   public bool coroutine;
    void Start()
    {
        coroutine = false;
        // StartCoroutine(GameFlow());
    }

    // Update is called once per frame
    void Update()
    {
        JS = JS_sub.JointState;
        if(!coroutine && target_sub.target_ok){
            print("statcoroutine");
        StartCoroutine(GameFlow());}
        // if (JS[0]!=0f && !js_init)
        // {


        // }
    }

    IEnumerator GameFlow()
    {
        coroutine=true;
        // 执行第一件事 initialization
            Debug.Log("第一件事开始");
            js_init.enabled = true;
            JS_pub.enabled = false;
            agent.enabled=false;
            

            yield return new WaitUntil(() => !js_init.enabled);
            
            yield return new WaitForSeconds(1); // 假设第一件事花费2秒
            Debug.Log("第一件事完成");
        
        
        
        // 执行第二件事 grasp first targ
        Debug.Log("第二件事开始");
        agent.target_turn = 1;
        agent.enabled = true;
        
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !agent.enabled);
        yield return new WaitForSeconds(2); // 假设第二件事花费2秒
        Debug.Log("第二件事完成");

         // 执行第3件事
        Debug.Log("第3件事开始");
        after_reach.target_turn=1;
        after_reach.first=false;
        after_reach.enabled = true;
        print("?"+after_reach.enabled);
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !after_reach.enabled);
        yield return new WaitForSeconds(2); // 假设第3件事花费2秒
        Debug.Log("第3件事完成");

           // 执行第4件事, grasp the 2nd targ
        Debug.Log("第4件事开始");
       
        agent.target_turn = 2;
        agent.enabled = true;
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !agent.enabled);
        yield return new WaitForSeconds(2); // 假设第二件事花费2秒
        Debug.Log("第4件事完成");

         // 执行第5件事
        Debug.Log("第5件事开始");
        after_reach.target_turn=2;
        after_reach.first=false;
        after_reach.enabled = true;
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !after_reach.enabled);
        yield return new WaitForSeconds(2); // 假设第3件事花费2秒
        Debug.Log("第5件事完成");

           // 执行第6件事, grasp the 3rd targ
        Debug.Log("第6件事开始");
        agent.target_turn = 3;
        agent.enabled = true;
        
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !agent.enabled);
        yield return new WaitForSeconds(2); // 假设第二件事花费2秒
        Debug.Log("第6件事完成");

         // 执行第7件事
        Debug.Log("第7件事开始");
        after_reach.target_turn=3;
        after_reach.first=false;
        after_reach.enabled = true;
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !after_reach.enabled);
        yield return new WaitForSeconds(2); // 假设第3件事花费2秒
        Debug.Log("第7件事完成");

           // 执行第8件事, grasp the 4rd targ
        Debug.Log("第6件事开始");
        agent.target_turn = 4;
        agent.enabled = true;
        
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !agent.enabled);
        yield return new WaitForSeconds(2); // 假设第二件事花费2秒
        Debug.Log("第6件事完成");

         // 执行第9件事
        Debug.Log("第7件事开始");
        after_reach.target_turn=4;
        after_reach.first=false;
        after_reach.enabled = true;
        JS_pub.enabled = true;
        yield return new WaitUntil(() => !after_reach.enabled);
        yield return new WaitForSeconds(2); // 假设第3件事花费2秒
        Debug.Log("第7件事完成");
        
        
        
    }
}
