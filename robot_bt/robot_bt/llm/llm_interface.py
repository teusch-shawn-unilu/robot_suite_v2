import os, openai, json
from plugin_base import PluginNode
from std_srvs.srv import Trigger

class LLMPlugin(PluginNode):
    """
    LLM wrapper - it listens on a service and
    drops the reply onto a latched topic for the BT to consume.
    """
    def __init__(self):
        super().__init__("llm_plugin")
        self.declare_parameter("model", "gpt-3.5-turbo")
        self.declare_parameter("apikey", os.getenv("OPENAI_API_KEY", ""))
        self._model = self.get_parameter("model").value

        self.srv = self.create_service(
            Trigger, "/llm/ask", self._srv_cb
        )
        self.pub = self.create_publisher(
            std_msgs.msg.String, "/llm/answer", 1
        )

    def _srv_cb(self, request, response):
        prompt = request.data
        try:
            reply = openai.ChatCompletion.create(
                model=self._model,
                messages=[{"role": "user", "content": prompt}],
            ).choices[0].message.content
            self.pub.publish(std_msgs.msg.String(data=reply))
            response.success = True
            response.message = reply
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    # behaviour‑tree never calls tick() – standalone only
